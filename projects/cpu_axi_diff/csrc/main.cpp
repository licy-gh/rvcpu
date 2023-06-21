// --xuezhen--
//rvcpu-test.cpp
#include <verilated.h>          
#include <verilated_vcd_c.h>    
#include <iostream>
#include <fstream>
#include "Vrvcpu.h"

using namespace std;

static Vrvcpu* top;
static VerilatedVcdC* tfp;
static vluint64_t main_time = 0;
static const vluint64_t sim_time = 1000;

// inst.bin
// inst 0: 1 + zero = reg1 1+0=1
// inst 1: 2 + zero = reg1 2+0=2
// inst 2: 1 + reg1 = reg1 1+2=3
int inst_rom[65536];

void read_inst( char* filename)
{
  FILE *fp = fopen(filename, "rb");
  if( fp == NULL ) {
		printf( "Can not open this file!\n" );
		exit(1);
  }
  
  fseek(fp, 0, SEEK_END);
  size_t size = ftell(fp);
  fseek(fp, 0, SEEK_SET);
  size = fread(inst_rom, size, 1, fp);
  fclose(fp);
}

int main(int argc, char **argv)
{
	char filename[100];
	printf("Please enter your filename~\n");
	cin >> filename;
	read_inst(filename);

  // initialization
  Verilated::commandArgs(argc, argv);
  Verilated::traceEverOn(true);

	top = new Vrvcpu;
  tfp = new VerilatedVcdC;

  top->trace(tfp, 99);
  tfp->open("top.vcd");
	
	while( !Verilated::gotFinish() && main_time < sim_time )
	{
	  if( main_time % 10 == 0 ) top->cpu_clk_50M = 0;
	  if( main_time % 10 == 5 ) top->cpu_clk_50M = 1;
		  
	  if( main_time < 10 )
	  {
		top->cpu_rst_n = 1;
	  }
	  else
	  {
	    top->cpu_rst_n = 0;
		if( main_time % 10 == 5 )
		  top->inst = (top->ice == 1) ? inst_rom[ (top->iaddr) >> 2 ] : 0;
	  }
	  top->eval();
	  tfp->dump(main_time);
	  main_time++;
	}
		
  // clean
  tfp->close();
  delete top;
  delete tfp;
  exit(0);
  return 0;
}
