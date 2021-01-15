#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

#include <sys/time.h>
#include <unistd.h>

#include "riscvISA.h"
#include "basic_simulator.h"
#include "core.h"
#include "elfFile.h"

#define DEBUG 0

BasicSimulator::BasicSimulator(const char* binaryFile, std::vector<std::string> args, const char* inFile,
                               const char* outFile, const char* tFile, const char* sFile)
{

  memset((char*)&core, 0, sizeof(Core));

  mem.reserve(DRAM_SIZE >> 2);

  core.im = new SimpleMemory<4>(mem.data());
  core.dm = new SimpleMemory<4>(mem.data());
  
  // core.im = new CacheMemory<4, 16, 64>(new SimpleMemory<4>(mem.data()), false);
  // core.dm = new CacheMemory<4, 16, 64>(new SimpleMemory<4>(mem.data()), false);

  heapAddress = 0;

  inputFile  = stdin;
  outputFile = stdout;
  traceFile  = stderr;
  signatureFile = NULL;
  
  if (inFile)
    inputFile = fopen(inFile, "rb");
  if (outFile)
    outputFile = fopen(outFile, "wb");
  if (tFile)
    traceFile = fopen(tFile, "wb");
  if(sFile)
    signatureFile = fopen(sFile, "wb");

  //****************************************************************************
  // Populate memory using ELF file
  readElf(binaryFile);

  if(DEBUG){
    printf("Start Symbol Reading done.\n");
  }

  pushArgsOnStack(args);

  if(DEBUG){
    printf("Populate Data Memory done.\n");
  }
  
  core.regFile[2] = STACK_INIT;
}

void BasicSimulator::readElf(const char *binaryFile){
  ElfFile elfFile(binaryFile);
  for(auto const &section : elfFile.sectionTable){
    if(section.address != 0){
      for (unsigned i = 0; i < section.size; i++)
        setByte(section.address + i, elfFile.content[section.offset + i]);
     
       // update the size of the heap
       if (section.name != ".text" && section.name != ".text.init") {
         if (section.address + section.size > heapAddress)
           heapAddress = section.address + section.size;
       }
    }
  }
  
  core.pc = find_by_name(elfFile.symbols, "_start").offset;
  if (signatureFile != NULL){
    begin_signature = find_by_name(elfFile.symbols, "begin_signature").offset;
    end_signature = find_by_name(elfFile.symbols, "end_signature").offset;
  }
}

void BasicSimulator::pushArgsOnStack(const std::vector<std::string> args){
  unsigned int argc = args.size();
 
  mem[STACK_INIT >> 2] = argc;  

  auto currentPlaceStrings = STACK_INIT + 4 + 4 * argc;
  for (unsigned oneArg = 0; oneArg < argc; oneArg++) {
    mem[(STACK_INIT + 4 * oneArg + 4) >> 2] = currentPlaceStrings;
    
    int oneCharIndex = 0;
    for(const auto c : args[oneArg]){
      setByte(currentPlaceStrings + oneCharIndex, c);
      oneCharIndex++;
    }
    setByte(currentPlaceStrings + oneCharIndex, 0);
    currentPlaceStrings += oneCharIndex + 1;
  }
} 

BasicSimulator::~BasicSimulator()
{
  if (inputFile)
    fclose(inputFile);
  if (outputFile)
    fclose(outputFile);
  if (traceFile)
    fclose(traceFile);
  if(signatureFile)
    fclose(signatureFile);
}

void BasicSimulator::printCycle()
{
  if(DEBUG){ 
    if (!core.stallSignals[0] && !core.stallIm && !core.stallDm) {
      printf("Debug trace : %x ", (unsigned int)core.ftoDC.pc);
      std::cout << printDecodedInstrRISCV(core.ftoDC.instruction);

      for (const auto &reg : core.regFile)
        printf("%x  ", (unsigned int)reg);
      std::cout << std::endl;
    }
  }
}

void BasicSimulator::printEnd()
{
  /*
  RISCV-COMPLIANCE Ending Routine to get the signature 
  stored between begin_signature and end_signature. 
  */
  if(signatureFile != NULL){
    const auto addr_offset = 4;
    const auto begin_offset = ((begin_signature)%4); // correct address alignement
    
    if(DEBUG){
      printf("BEGIN/END_SIGNATURE: %x/%x (%x)", begin_signature, end_signature, begin_offset);
    }
    
    //memory read
    for (auto wordNumber = begin_signature - begin_offset; wordNumber < end_signature - begin_offset; wordNumber+=addr_offset)
    {
      fprintf(signatureFile, "%08x\n", (unsigned int)this->ldw(wordNumber));
    }
  }
}

void BasicSimulator::setByte(unsigned addr, ac_int<8, true> value){
  mem[addr >> 2].set_slc((addr % 4) * 8, value); 
}

// Function for handling memory accesses
void BasicSimulator::stb(ac_int<32, false> addr, ac_int<8, true> value)
{
  ac_int<32, false> wordRes = 0;
  bool stall                = true;
  while (stall)
    core.dm->process(addr, BYTE, STORE, value, wordRes, stall);
}

void BasicSimulator::sth(ac_int<32, false> addr, ac_int<16, true> value)
{
  this->stb(addr + 1, value.slc<8>(8));
  this->stb(addr + 0, value.slc<8>(0));
}

void BasicSimulator::stw(ac_int<32, false> addr, ac_int<32, true> value)
{
  this->stb(addr + 3, value.slc<8>(24));
  this->stb(addr + 2, value.slc<8>(16));
  this->stb(addr + 1, value.slc<8>(8));
  this->stb(addr + 0, value.slc<8>(0));
}

void BasicSimulator::std(ac_int<32, false> addr, ac_int<64, true> value)
{
  this->stb(addr + 7, value.slc<8>(56));
  this->stb(addr + 6, value.slc<8>(48));
  this->stb(addr + 5, value.slc<8>(40));
  this->stb(addr + 4, value.slc<8>(32));
  this->stb(addr + 3, value.slc<8>(24));
  this->stb(addr + 2, value.slc<8>(16));
  this->stb(addr + 1, value.slc<8>(8));
  this->stb(addr + 0, value.slc<8>(0));
}

ac_int<8, true> BasicSimulator::ldb(ac_int<32, false> addr)
{
  ac_int<8, true> result;
  result                    = mem[addr >> 2].slc<8>(((int)addr.slc<2>(0)) << 3);
  ac_int<32, false> wordRes = 0;
  bool stall                = true;
  while (stall)
    core.dm->process(addr, BYTE_U, LOAD, 0, wordRes, stall);

  result = wordRes.slc<8>(0);
  return result;
}

// Little endian version
ac_int<16, true> BasicSimulator::ldh(ac_int<32, false> addr)
{
  ac_int<16, true> result = 0;
  result.set_slc(8, this->ldb(addr + 1));
  result.set_slc(0, this->ldb(addr));
  return result;
}

ac_int<32, true> BasicSimulator::ldw(ac_int<32, false> addr)
{
  ac_int<32, true> result = 0;
  result.set_slc(24, this->ldb(addr + 3));
  result.set_slc(16, this->ldb(addr + 2));
  result.set_slc(8, this->ldb(addr + 1));
  result.set_slc(0, this->ldb(addr));
  return result;
}

ac_int<32, true> BasicSimulator::ldd(ac_int<32, false> addr)
{
  ac_int<32, true> result = 0;
  result.set_slc(56, this->ldb(addr + 7));
  result.set_slc(48, this->ldb(addr + 6));
  result.set_slc(40, this->ldb(addr + 5));
  result.set_slc(32, this->ldb(addr + 4));
  result.set_slc(24, this->ldb(addr + 3));
  result.set_slc(16, this->ldb(addr + 2));
  result.set_slc(8, this->ldb(addr + 1));
  result.set_slc(0, this->ldb(addr));

  return result;
}

/********************************************************************************************************************
**  Software emulation of system calls.
**
** Currently all system calls are solved in the simulator. The function solveSyscall check the opcode in the
** extoMem pipeline registers and verifies whether it is a syscall or not. If it is, they solve the forwarding,
** and switch to the correct function according to reg[17].
*********************************************************************************************************************/
void BasicSimulator::solveSyscall()
{

  if ((core.extoMem.opCode == RISCV_SYSTEM) && core.extoMem.instruction.slc<12>(20) == 0 && !core.stallSignals[2] &&
      !core.stallIm && !core.stallDm) {

    ac_int<32, true> syscallId = core.regFile[17];
    ac_int<32, true> arg1      = core.regFile[10];
    ac_int<32, true> arg2      = core.regFile[11];
    ac_int<32, true> arg3      = core.regFile[12];
    ac_int<32, true> arg4      = core.regFile[13];

    if (core.memtoWB.useRd && core.memtoWB.we && !core.stallSignals[3]) {
      if (core.memtoWB.rd == 10)
        arg1 = core.memtoWB.result;
      else if (core.memtoWB.rd == 11)
        arg2 = core.memtoWB.result;
      else if (core.memtoWB.rd == 12)
        arg3 = core.memtoWB.result;
      else if (core.memtoWB.rd == 13)
        arg4 = core.memtoWB.result;
      else if (core.memtoWB.rd == 17)
        syscallId = core.memtoWB.result;
    }

    ac_int<32, true> result = 0;

    switch (syscallId) {
      case SYS_exit:
        exitFlag = 1; // Currently we break on ECALL
        break;
      case SYS_read:
        result = this->doRead(arg1, arg2, arg3);
        break;
      case SYS_write:
        result = this->doWrite(arg1, arg2, arg3);
        break;
      case SYS_brk:
        result = this->doSbrk(arg1);
        break;
      case SYS_open:
        result = this->doOpen(arg1, arg2, arg3);
        break;
      case SYS_openat:
        result = this->doOpenat(arg1, arg2, arg3, arg4);
        break;
      case SYS_lseek:
        result = this->doLseek(arg1, arg2, arg3);
        break;
      case SYS_close:
        result = this->doClose(arg1);
        break;
      case SYS_fstat:
        result = this->doFstat(arg1, arg2);
        break;
      case SYS_stat:
        result = this->doStat(arg1, arg2);
        break;
      case SYS_gettimeofday:
        result = this->doGettimeofday(arg1);
        break;
      case SYS_unlink:
        result = this->doUnlink(arg1);
        break;
      case SYS_exit_group:
        fprintf(stderr, "Syscall : SYS_exit_group\n");
        exitFlag = 1;
        break;
      case SYS_getpid:
        fprintf(stderr, "Syscall : SYS_getpid\n");
        exitFlag = 1;
        break;
      case SYS_kill:
        fprintf(stderr, "Syscall : SYS_kill\n");
        exitFlag = 1;
        break;
      case SYS_link:
        fprintf(stderr, "Syscall : SYS_link\n");
        exitFlag = 1;
        break;
      case SYS_mkdir:
        fprintf(stderr, "Syscall : SYS_mkdir\n");
        exitFlag = 1;
        break;
      case SYS_chdir:
        fprintf(stderr, "Syscall : SYS_chdir\n");
        exitFlag = 1;
        break;
      case SYS_getcwd:
        fprintf(stderr, "Syscall : SYS_getcwd\n");
        exitFlag = 1;
        break;
      case SYS_lstat:
        fprintf(stderr, "Syscall : SYS_lstat\n");
        exitFlag = 1;
        break;
      case SYS_fstatat:
        fprintf(stderr, "Syscall : SYS_fstatat\n");
        exitFlag = 1;
        break;
      case SYS_access:
        fprintf(stderr, "Syscall : SYS_access\n");
        exitFlag = 1;
        break;
      case SYS_faccessat:
        fprintf(stderr, "Syscall : SYS_faccessat\n");
        exitFlag = 1;
        break;
      case SYS_pread:
        fprintf(stderr, "Syscall : SYS_pread\n");
        exitFlag = 1;
        break;
      case SYS_pwrite:
        fprintf(stderr, "Syscall : SYS_pwrite\n");
        exitFlag = 1;
        break;
      case SYS_uname:
        fprintf(stderr, "Syscall : SYS_uname\n");
        exitFlag = 1;
        break;
      case SYS_getuid:
        fprintf(stderr, "Syscall : SYS_getuid\n");
        exitFlag = 1;
        break;
      case SYS_geteuid:
        fprintf(stderr, "Syscall : SYS_geteuid\n");
        exitFlag = 1;
        break;
      case SYS_getgid:
        fprintf(stderr, "Syscall : SYS_getgid\n");
        exitFlag = 1;
        break;
      case SYS_getegid:
        fprintf(stderr, "Syscall : SYS_getegid\n");
        exitFlag = 1;
        break;
      case SYS_mmap:
        fprintf(stderr, "Syscall : SYS_mmap\n");
        exitFlag = 1;
        break;
      case SYS_munmap:
        fprintf(stderr, "Syscall : SYS_munmap\n");
        exitFlag = 1;
        break;
      case SYS_mremap:
        fprintf(stderr, "Syscall : SYS_mremap\n");
        exitFlag = 1;
        break;
      case SYS_time:
        fprintf(stderr, "Syscall : SYS_time\n");
        exitFlag = 1;
        break;
      case SYS_getmainvars:
        fprintf(stderr, "Syscall : SYS_getmainvars\n");
        exitFlag = 1;
        break;
      case SYS_rt_sigaction:
        fprintf(stderr, "Syscall : SYS_rt_sigaction\n");
        exitFlag = 1;
        break;
      case SYS_writev:
        fprintf(stderr, "Syscall : SYS_writev\n");
        exitFlag = 1;
        break;
      case SYS_times:
        fprintf(stderr, "Syscall : SYS_times\n");
        exitFlag = 1;
        break;
      case SYS_fcntl:
        fprintf(stderr, "Syscall : SYS_fcntl\n");
        exitFlag = 1;
        break;
      case SYS_getdents:
        fprintf(stderr, "Syscall : SYS_getdents\n");
        exitFlag = 1;
        break;
      case SYS_dup:
        fprintf(stderr, "Syscall : SYS_dup\n");
        exitFlag = 1;
        break;

        // Custom syscalls
      case SYS_threadstart:
        result = 0;
        break;
      case SYS_nbcore:
        result = 1;
        break;

      default:
        fprintf(stderr, "Syscall : Unknown system call, %d (%x) with arguments :\n", syscallId.to_int(),
                syscallId.to_int());
        fprintf(stderr, "%d (%x)\n%d (%x)\n%d (%x)\n%d (%x)\n", arg1.to_int(), arg1.to_int(), arg2.to_int(),
                arg2.to_int(), arg3.to_int(), arg3.to_int(), arg4.to_int(), arg4.to_int());
        exitFlag = 1;
        break;
    }

    // We write the result and forward
    core.memtoWB.result = result;
    core.memtoWB.rd     = 10;
    core.memtoWB.useRd  = 1;

    if (core.dctoEx.useRs1 && (core.dctoEx.rs1 == 10))
      core.dctoEx.lhs = result;
    if (core.dctoEx.useRs2 && (core.dctoEx.rs2 == 10))
      core.dctoEx.rhs = result;
    if (core.dctoEx.useRs3 && (core.dctoEx.rs3 == 10))
      core.dctoEx.datac = result;
  }
}

ac_int<32, true> BasicSimulator::doRead(ac_int<32, false> file, ac_int<32, false> bufferAddr, ac_int<32, false> size)
{
  char* localBuffer = new char[size.to_int()];
  ac_int<32, true> result;

  if (file == 0 && inputFile)
    result = read(inputFile->_fileno, localBuffer, size);
  else
    result = read(file, localBuffer, size);

  for (int i(0); i < result; i++) {
    this->stb(bufferAddr + i, localBuffer[i]);
  }

  delete[] localBuffer;
  return result;
}

ac_int<32, true> BasicSimulator::doWrite(ac_int<32, false> file, ac_int<32, false> bufferAddr, ac_int<32, false> size)
{
  char* localBuffer = new char[size.to_int()];

  for (int i = 0; i < size; i++)
    localBuffer[i] = this->ldb(bufferAddr + i);

  ac_int<32, true> result = 0;
  if (file == 1 && outputFile) // 3 is the first available file descriptor
  {
    fflush(stdout);
    result = write(outputFile->_fileno, localBuffer, size);
  } else {
    if (file == 1)
      fflush(stdout);
    else if (file == 2)
      fflush(stderr);
    result = write(file, localBuffer, size);
  }
  delete[] localBuffer;
  return result;
}

ac_int<32, true> BasicSimulator::doFstat(ac_int<32, false> file, ac_int<32, false> stataddr)
{
  ac_int<32, true> result = 0;
  struct stat filestat    = {0};

  if (file != 1)
    result = fstat(file, &filestat);

  std(stataddr, filestat.st_dev);               // unsigned long long
  std(stataddr + 8, filestat.st_ino);           // unsigned long long
  stw(stataddr + 16, filestat.st_mode);         // unsigned int
  stw(stataddr + 20, filestat.st_nlink);        // unsigned int
  stw(stataddr + 24, filestat.st_uid);          // unsigned int
  stw(stataddr + 28, filestat.st_gid);          // unsigned int
  std(stataddr + 32, filestat.st_rdev);         // unsigned long long
  std(stataddr + 40, filestat.__pad0);          // unsigned long long
  std(stataddr + 48, filestat.st_size);         // long long
  stw(stataddr + 56, filestat.st_blksize);      // int
  stw(stataddr + 60, filestat.__pad0);          // int
  std(stataddr + 64, filestat.st_blocks);       // long long
  stw(stataddr + 72, filestat.st_atim.tv_sec);  // long
  stw(stataddr + 76, filestat.st_atim.tv_nsec); // long
  stw(stataddr + 80, filestat.st_mtim.tv_sec);  // long
  stw(stataddr + 84, filestat.st_mtim.tv_nsec); // long
  stw(stataddr + 88, filestat.st_ctim.tv_sec);  // long
  stw(stataddr + 92, filestat.st_ctim.tv_nsec); // long
  stw(stataddr + 96, filestat.__pad0);          // long
  stw(stataddr + 100, filestat.__pad0);         // long

  return result;
}

ac_int<32, true> BasicSimulator::doOpen(ac_int<32, false> path, ac_int<32, false> flags, ac_int<32, false> mode)
{
  int oneStringElement = this->ldb(path);
  int index            = 0;
  while (oneStringElement != 0) {
    index++;
    oneStringElement = this->ldb(path + index);
  }

  int pathSize = index + 1;

  char* localPath = new char[pathSize + 1];
  for (int i = 0; i < pathSize; i++)
    localPath[i] = this->ldb(path + i);
  localPath[pathSize] = '\0';

  // convert riscv flags to unix flags
  int riscvflags = flags;
  std::string str;
  if (riscvflags & SYS_O_WRONLY)
    str += "WRONLY, ";
  else if (riscvflags & SYS_O_RDWR)
    str += "RDWR, ";
  else
    str += "RDONLY, ";
  int unixflags = riscvflags & 3; // O_RDONLY, O_WRITE, O_RDWR are the same
  riscvflags ^= unixflags;
  if (riscvflags & SYS_O_APPEND) {
    unixflags |= O_APPEND;
    riscvflags ^= SYS_O_APPEND;
    str += "APPEND, ";
  }
  if (riscvflags & SYS_O_CREAT) {
    unixflags |= O_CREAT;
    riscvflags ^= SYS_O_CREAT;
    str += "CREAT, ";
  }
  if (riscvflags & SYS_O_TRUNC) {
    unixflags |= O_TRUNC;
    riscvflags ^= SYS_O_TRUNC;
    str += "TRUNC, ";
  }
  if (riscvflags & SYS_O_EXCL) {
    unixflags |= O_EXCL;
    riscvflags ^= SYS_O_EXCL;
    str += "EXCL, ";
  }
  if (riscvflags & SYS_O_SYNC) {
    unixflags |= O_SYNC;
    riscvflags ^= SYS_O_SYNC;
    str += "SYNC, ";
  }
  if (riscvflags & SYS_O_NONBLOCK) {
    unixflags |= O_NONBLOCK;
    riscvflags ^= SYS_O_NONBLOCK;
    str += "NONBLOCK, ";
  }
  if (riscvflags & SYS_O_NOCTTY) {
    unixflags |= O_NOCTTY;
    riscvflags ^= SYS_O_NOCTTY;
    str += "NOCTTY";
  }
  int result = open(localPath, unixflags, mode.to_int());

  delete[] localPath;
  return result;
}

ac_int<32, true> BasicSimulator::doOpenat(ac_int<32, false> dir, ac_int<32, false> path, ac_int<32, false> flags,
                                          ac_int<32, false> mode)
{
  fprintf(stderr, "Syscall : SYS_openat not implemented yet...\n");
  exit(-1);
}

ac_int<32, true> BasicSimulator::doClose(ac_int<32, false> file)
{
  if (file > 2) // don't close simulator's stdin, stdout & stderr
  {
    return close(file);
  }

  return 0;
}

ac_int<32, true> BasicSimulator::doLseek(ac_int<32, false> file, ac_int<32, false> ptr, ac_int<32, false> dir)
{
  int result = lseek(file, ptr, dir);
  return result;
}

ac_int<32, true> BasicSimulator::doStat(ac_int<32, false> filename, ac_int<32, false> stataddr)
{
  int oneStringElement = this->ldb(filename);
  int index            = 0;
  while (oneStringElement != 0) {
    index++;
    oneStringElement = this->ldb(filename + index);
  }

  int pathSize = index + 1;

  char* localPath = new char[pathSize + 1];
  for (int i = 0; i < pathSize; i++)
    localPath[i] = this->ldb(filename + i);
  localPath[pathSize] = '\0';

  struct stat filestat;
  int result = stat(localPath, &filestat);

  std(stataddr, filestat.st_dev);               // unsigned long long
  std(stataddr + 8, filestat.st_ino);           // unsigned long long
  stw(stataddr + 16, filestat.st_mode);         // unsigned int
  stw(stataddr + 20, filestat.st_nlink);        // unsigned int
  stw(stataddr + 24, filestat.st_uid);          // unsigned int
  stw(stataddr + 28, filestat.st_gid);          // unsigned int
  std(stataddr + 32, filestat.st_rdev);         // unsigned long long
  std(stataddr + 40, filestat.__pad0);          // unsigned long long
  std(stataddr + 48, filestat.st_size);         // long long
  stw(stataddr + 56, filestat.st_blksize);      // int
  stw(stataddr + 60, filestat.__pad0);          // int
  std(stataddr + 64, filestat.st_blocks);       // long long
  stw(stataddr + 72, filestat.st_atim.tv_sec);  // long
  stw(stataddr + 76, filestat.st_atim.tv_nsec); // long
  stw(stataddr + 80, filestat.st_mtim.tv_sec);  // long
  stw(stataddr + 84, filestat.st_mtim.tv_nsec); // long
  stw(stataddr + 88, filestat.st_ctim.tv_sec);  // long
  stw(stataddr + 92, filestat.st_ctim.tv_nsec); // long
  stw(stataddr + 96, filestat.__pad0);          // long
  stw(stataddr + 100, filestat.__pad0);         // long

  delete[] localPath;
  return result;
}

ac_int<32, true> BasicSimulator::doSbrk(ac_int<32, false> value)
{
  ac_int<32, true> result;
  if (value == 0) {
    result = heapAddress;
  } else {
    heapAddress = value;
    result      = value;
  }

  return result;
}

ac_int<32, true> BasicSimulator::doGettimeofday(ac_int<32, false> timeValPtr)
{
  struct timeval oneTimeVal;
  int result = gettimeofday(&oneTimeVal, NULL);

  this->stw(timeValPtr, oneTimeVal.tv_sec);
  this->stw(timeValPtr + 4, oneTimeVal.tv_usec);

  return result;
}

ac_int<32, true> BasicSimulator::doUnlink(ac_int<32, false> path)
{
  int oneStringElement = this->ldb(path);
  int index            = 0;
  while (oneStringElement != '\0') {
    index++;
    oneStringElement = this->ldb(path + index);
  }

  int pathSize = index + 1;

  char* localPath = new char[pathSize + 1];
  for (int i = 0; i < pathSize; i++)
    localPath[i] = this->ldb(path + i);
  localPath[pathSize] = '\0';

  int result = unlink(localPath);

  delete[] localPath;
  return result;
}
