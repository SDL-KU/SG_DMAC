/******************************************************************************
Copyright (c) 2019 SoC Design Laboratory, Konkuk University, South Korea
            2019 Bit Engneering Laboratory, Pusan National University, South Korea
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met: redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer;
redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution;
neither the name of the copyright holders nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Authors: SoC Design Laboratory, Konkuk University,
         Bit Engneering Laboratory, Pusan National University

Revision History
2019.08.31: SoC Design Laboratory, Konkuk University.
*******************************************************************************/
#ifndef DSE_H
#define DSE_H

#include "systemc.h"
#include <bitset>

//////////////////////////////////////////////////////////////////////////
//////////LAYER CONFIG////////////////////////////////////////////////////
/////////// lp: Layer Param //////////
/////////// dp: Design Param //////////
int   lp_B = 60;
int   lp_M = 384;
int   lp_C = 256;
int   lp_R = 3;
int   lp_S = 3;
int   lp_U = 1;
int   lp_E = 13;
int   lp_F = 13;
int   lp_H = ((lp_E - 1) * 1 + lp_R);
int   lp_W = ((lp_F - 1) * 1 + lp_S);
int   dp_TM = 64;
int   dp_TC = 100;
int   dp_TE = 8;//13;
int   dp_TF = 8;//13;
int   dp_TB = 1;
int   dp_TH = ((dp_TE - 1) * 1 + lp_R);
int   dp_TW = ((dp_TF - 1) * 1 + lp_S);

int   dp_UM = 64;
int   dp_UC = 2;

#define   num_reuse_i     1
#define   num_reuse_f     1
#define   num_reuse_o     ((lp_C+dp_TC-1)/dp_TC)
#define   num_continue_i  1
#define   num_continue_f  1
#define   num_continue_o  ((lp_C+dp_TC-1)/dp_TC)
#define   TOTAL_MACs      (lp_B * lp_M * lp_C * lp_E * lp_F * lp_R * lp_S)
#define   NUM_TOTAL_PASS   ( (lp_B+dp_TB-1) / dp_TB)*( (lp_M+dp_TM-1) / dp_TM)*( (lp_C+dp_TC-1) / dp_TC)*( (lp_E+dp_TE-1) / dp_TE)*( (lp_F+dp_TF-1) / dp_TF)

#define SRAM_LEAKAGE 7.395312e-05
#define SRAM_DYNAMIC 4.505785e-03
#define REG_LEAKAGE 7.395312e-05
#define REG_DYNAMIC 4.505785e-03
#define MULT_LEAKAGE 4.831147e-02
#define MULT_DYNAMIC 4.777429e+00
#define ADD_LEAKAGE 2.380803e-03
#define ADD_DYNAMIC 4.287582e-02


int no_pass_executed;
//#define BASIC_LAYOUT_REDANDUNT
#define BASIC_LAYOUT_INTERLEAVE
// #define IDEAL_LAYOUT

#define ONLY_REPRESENTATIVE_PASS

//////////////////////////////////////////////////////////////////////////
//////////DMA CONFIG//////////////////////////////////////////////////////
// #define UDSG_MODE


#define SG_MODE
// #define base_reg_polling
// #define OPT_SG_V3
int SG_R_NMOT = 3;
int SG_W_NMOT = 1;

#define USE_SETSDATA_TO_WRITE_DATA
#ifdef SG_MODE
int CFG_FIFO_SIZE = 2000;
#elif defined UDSG_MODE
int CFG_FIFO_SIZE = 20000;
#else
int CFG_FIFO_SIZE = 1;
#endif
int DMA_MAX_BURST_LENGTH = 128;
int DMA_MAX_BURST_NUM = 2;
int DATA_FIFO_SIZE      =DMA_MAX_BURST_LENGTH*10 ;// burst length * 2

#define MIN(a,b)        ((a < b ? a : ((b == 0)? 1 : b )))
#define MAX(a,b)        (a > b ? a : b)

#define IFMAP_DDR_ADDR      0
#define FMAP_DDR_ADDR       (IFMAP_DDR_ADDR + lp_B * lp_C * lp_H * lp_W*sizeof(int))
#define OFMAP_DDR_ADDR      (FMAP_DDR_ADDR + lp_C * lp_M * lp_R * lp_S*sizeof(int))

#define IFMAP_BASE			(OFMAP_DDR_ADDR + lp_B * lp_M * lp_E * lp_F*sizeof(int))
#define FMAP_BASE			   (IFMAP_BASE + dp_TB * dp_TC * dp_TH * dp_TW*(int)ceil((float)lp_H/(float)dp_TH)*(int)ceil((float)lp_W/(float)dp_TW)*(int)ceil((float)lp_C/(float)dp_TC)*(int)ceil((float)lp_B/(float)dp_TB)*sizeof(int))
#define OFMAP_BASE			(FMAP_BASE+dp_TM*dp_TC * lp_R * lp_S*(int)ceil((float)lp_C/(float)dp_TC)*(int)ceil((float)lp_M/(float)dp_TM)*sizeof(int))

#define IFMAP_GLB_ADDR      0
#define FMAP_GLB_ADDR       0
#define OFMAP_GLB_ADDR      0

//////////////////////////////////////////////////////////////////////////
///////////////////////////BUS CONFIG/////////////////////////////////////
 // #define SDL_BUS

// #define ROUND_ROBIN
#define FIXED_PRIORITY

#ifdef SG_MODE
	#define NUM_OF_MASTER     6
#else
	#define NUM_OF_MASTER     3
#endif

#define NUM_OF_SLAVE      1


//////////////////////////////////////////////////////////////////////////
///////////////////////////TIME INFORMATION////////////////////////////////
int DMA_SET_GO_LATENCY = 80;
int BUSY_CHECK_PREIOD = 16;
int AW2WREADY_LATENCY = 3;
int DDR_FW_PATH_LATENCY = 7;
int DDR_R_BW_PATH_LATENCY = 4;
int DDR_W_BW_PATH_LATENCY = 3;
int MULTIPLE_OUT_LATENCY = 5;
int SG_RLAST_TO_ADDR_REQ_LATENCY = 5;
int SG_RLAST_TO_NEXT_SG_ADDR_REQ_LATENCY = 9;
int TRANSFER_DONE_TO_NEXT_SG_ADDR_REQ_LATENCY = 17;
int TRANSFER_DONE_TO_SG_AW_REQ_LATENCY = 15;
int SG_AW_HANDSHAKE_TO_W_REQ_LATENCY = 3;
int READ_ARBIT_LATENCY = 4;         //need to debug, only run when it is 4
int WRITE_ARBIT_LATENCY = 5;
int XBAR_DMA_DATA_PIPLINE = 2;
int RDMA_SIF_DATA_PIPLINE = 4;    //need to check which one is correct BL or 4 cycles
int WDMA_SIF_DATA_PIPLINE = 10;
int REARRANGE_LATENCY = 0;


float fSysCTimeInterval0 = 0;
float fSysCTimeInterval1 = 0;

int TimeInterval0 = 0;
int TimeInterval1 = 0;
int TimeInterval2 = 0;

float log_macps_rpass = 0;
float log_etc0;

// for beholder
float log_intv_a_dat[3] = {0,};
float log_intv_b_dat[3] = {0,}; 
float log_intv_c_dat[3] = {0,};

int dma_req_cnt[2] = {0,};
int dma_dat_cnt[2] = {0,};

bool* behold_ptr_acc_compute;

int cyc_rp_pass=0;

sc_event behold_pass_start;
sc_event behold_pass_end;

// void* behold_roi_pass_rep_start; //sc_event*
sc_event behold_roi_pass_rep_start;
sc_event behold_roi_pass_rep_end;



#define PROCESSING_PASS_DELAY ((dp_TE*dp_TF*lp_R*lp_S)+1)
//////////////////////////////////////////////////////////////////////////
//////////DDR CONFIG//////////////////////////////////////////////////////
#ifndef SDL_BUS
   #define OUT_OF_ORDER
#endif
#define DDR_MEMSIZE       512						//MB
#define SLAVE0_BASE    0
#define SLAVE1_BASE    DDR_MEMSIZE*1048576
#define DEVICE_INI      "ini/DDR3_micron_64M_8B_x4_sg15.ini"
#define DRMASIM2_ROOT     "../simplebus/dramsim2"

//////////////////////////////////////////////////////////////////////////
//////////PL CLOCK CONFIG//////////////////////////////////////////////////////
#define MHZ_               1000000
int PL_CLK_MHZ = 500;
unsigned int  PL_CLK_FREQ    =(PL_CLK_MHZ * MHZ_);         // Hz
float PL_CLK_PRD = (1000000000.0f/(float)PL_CLK_FREQ);
float DDR_CLK_PRD = 2.0f;

sc_time PL_CLK_PRD_SC = sc_time(PL_CLK_PRD,SC_NS);

#define CYCLES(x)       PL_CLK_PRD*(x), sc_core::SC_NS
#define D_CYCLES(x)       DDR_CLK_PRD*(x), sc_core::SC_NS

//////////////////////////////////////////////////////////////////////////
//////////ANYTHING ELSE///////////////////////////////////////////////////
// #define SIM_RESULT

void DSE_param_dependency_sync(void){
   dp_TH = ((dp_TE - 1) * 1 + lp_R);
   dp_TW = ((dp_TF - 1) * 1 + lp_S);
   PL_CLK_FREQ    =(PL_CLK_MHZ * MHZ_);         // Hz
   PL_CLK_PRD = (1000000000.0f/(float)PL_CLK_FREQ);
   PL_CLK_PRD_SC = sc_time(PL_CLK_PRD,SC_NS);
   DATA_FIFO_SIZE      =DMA_MAX_BURST_LENGTH*10 ;// burst length * 2
}

struct SG_DMA_DSE_PARAM {   
   int packet_addr               = 0x100;
   int bd_addr                = 0x30000;
   int packet_length             = 15;
   int line_offset               = 15;
   int macroblock_offset         = 225;
   int num_of_line_in_macroblock = 15;
   int num_of_macroblock         = 2;
   int num_of_packet          = num_of_line_in_macroblock*num_of_macroblock;
   int size_of_bd             = 8;
   int dse_option             =1;
   int tc_for_dse             =2;
} sg_dma_dse_param;


#endif //DSE_H
