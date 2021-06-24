#ifndef INCLUDE_BRANCHPREDICTOR_H_
#define INCLUDE_BRANCHPREDICTOR_H_

#include "logarithm.h"
#include <ac_int.h>

#ifdef DEBUG
#include <assert.h>
#include <iostream>
#include <queue>
#endif

template <class T> class BranchPredictorWrapper {
#ifdef DEBUG
  int missPredictions = 0;
  int processCount    = 0;
  int updateCount     = 0;
  bool predictions[2];
  int front = 0, back = 0;
#endif

public:
  void update(ac_int<32, false> pc, bool isBranch)
  {
    static_cast<T*>(this)->_update(pc, isBranch);
#ifdef DEBUG
    updateCount++;
    assert(updateCount + 1 >= processCount);
    bool pred = predictions[front];
    front ^= 1;
    missPredictions += isBranch == pred ? 0 : 1;
    std::cout << "pc: " << pc << "\n"
              << "branch: " << isBranch << "\n"
              << "predict: " << pred << "\n"
              << "miss rate: " << (float)missPredictions / updateCount << "\n";

#endif
  }

  void process(ac_int<32, false> pc, bool& isBranch)
  {
    static_cast<T*>(this)->_process(pc, isBranch);
#ifdef DEBUG
    processCount++;
    predictions[back] = isBranch;
    back ^= 1;
#endif
  }

  void undo()
  {
#ifdef DEBUG
    if (processCount > updateCount) {
      processCount--;
      front ^= 1;
    }
#endif
  }
};

template <int BITS, int ENTRIES>
class BitBranchPredictor : public BranchPredictorWrapper<BitBranchPredictor<BITS, ENTRIES> > {
  static const int LOG_ENTRIES = log2const<ENTRIES>::value;
  static const int NT_START    = (1 << BITS) - 1;
  static const int NT_FINAL    = (1 << BITS) >> 1;
  static const int T_START     = 0;
  static const int T_FINAL     = NT_FINAL - 1;

  ac_int<BITS, false> table[ENTRIES];

public:
  BitBranchPredictor()
  {
    for (int i = 0; i < ENTRIES; i++) {
      table[i] = T_START;
    }
  }

  void _update(ac_int<32, false> pc, bool isBranch)
  {
    ac_int<LOG_ENTRIES, false> index = pc.slc<LOG_ENTRIES>(2);

    if (isBranch) {
      table[index] -= table[index] != T_START ? 1 : 0;
    } else {
      table[index] += table[index] != NT_START ? 1 : 0;
    }
  }

  void _process(ac_int<32, false> pc, bool& isBranch)
  {
    ac_int<LOG_ENTRIES, false> index = pc.slc<LOG_ENTRIES>(2);
    isBranch                         = table[index] <= T_FINAL;
  }
};

template<int SIZE, int BITS, int ENTRIES, int THRESHOLD, int LR>
class PerceptronBranchPredictor : public BranchPredictorWrapper<PerceptronBranchPredictor<SIZE, BITS, ENTRIES, THRESHOLD, LR> > {
    static const int LOG_ENTRIES = log2const<ENTRIES>::value;
    static const int PERC_MAX    = (1 << (BITS - 1)) - 1;
    static const int PERC_MIN    = -(PERC_MAX + 1);
    static const int PERC_INC_TH = PERC_MAX - LR + 1;
    static const int PERC_DEC_TH = PERC_MIN + LR - 1;
    
    ac_int<BITS, true> perceptron[ENTRIES][SIZE+1];
    bool bht[SIZE];    // branch history table
    int  dp;           // dot product
    bool pd;           // prediction

public:
    PerceptronBranchPredictor() {
        for (int i = 0; i < ENTRIES; i++) {
            for (int j = 0; j < SIZE+1; j++) {
                perceptron[i][j] = 0;
            }
        }
        for (int i = 0; i < SIZE; i++) {
            bht[i] = 0;
        }
    }
    
    void _update(ac_int<32, false> pc, bool isBranch) {
        if (pd == isBranch && dp > THRESHOLD) return;
        ac_int<LOG_ENTRIES, false> index = pc.slc<LOG_ENTRIES>(0);
        if (isBranch) {
            if (perceptron[index][SIZE] < PERC_INC_TH) perceptron[index][SIZE] += LR;
        } else {
            if (perceptron[index][SIZE] > PERC_DEC_TH) perceptron[index][SIZE] -= LR;
        }
        
        update:for (int i = 0; i < SIZE; i++) {
            if (bht[i] == isBranch) {
                if (perceptron[index][i] < PERC_INC_TH) perceptron[index][i] += LR;
            } else {
                if (perceptron[index][i] > PERC_DEC_TH) perceptron[index][i] -= LR;
            }
        }

        shift_reg:for (int i = 0; i < SIZE-1; i++) {
            bht[i] = bht[i+1];
        }
        bht[SIZE-1] = isBranch;
    }

    void _predict(ac_int<LOG_ENTRIES, false> index) {
        dp = perceptron[index][SIZE];
        predict:for (int i = 0; i < SIZE; i++) {
            if (bht[i]) {
                dp += perceptron[index][i];
            } else {
                dp -= perceptron[index][i];
            }
        }
        pd = dp >= 0;
        if (!pd) dp = -dp;
    }
    
    void _process(ac_int<32, false> pc, bool& isBranch) {
        ac_int<LOG_ENTRIES, false> index = pc.slc<LOG_ENTRIES>(0);
        _predict(index);
        isBranch = pd;
    }
};

using BranchPredictor = BitBranchPredictor<2, 4>;

#endif /* INCLUDE_BRANCHPREDICTOR_H_ */
