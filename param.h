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
#ifndef USE_GPSOCKET
# define USE_GPSOCKET
#endif

//////////////////////////////////////////////////////////////////////////
//////////LAYER CONFIG////////////////////////////////////////////////////
/////////// dp: Design Param //////////
/////////// dp: Layer Param //////////
#define   dp_TM 					32
#define   dp_TC  					4
#define   dp_TE  					13
#define   dp_TF  				  13
#define   dp_TB           1
#define   lp_B  					1
#define   lp_M  					384
#define   lp_C  					256
#define   lp_R  					3
#define   lp_S  					3
#define   lp_U  					1
#define   lp_E  					13
#define   lp_F  					13
#define   lp_H  					((lp_E - 1) * 1 + lp_R)
#define   lp_W  					((lp_F - 1) * 1 + lp_S)
#define   num_reuse_i     1
#define   num_reuse_f     1
#define   num_reuse_o     lp_C/dp_TC
#define   num_continue_i  1
#define   num_continue_f  1
#define   num_continue_o  lp_C/dp_TC
#define   TOTAL_MACs      lp_M * lp_C * lp_E * lp_F * lp_R * lp_S
#define   InputActs       dp_TC * lp_H * lp_W
#define   OutputActs      dp_TM * dp_TE * dp_TF
#define   Weights         dp_TC * dp_TM * lp_R * lp_S


//////////////////////////////////////////////////////////////////////////
//////////DMA CONFIG//////////////////////////////////////////////////////
#define BURST_LENGTH 			4

#define USE_SETSDATA_TO_WRITE_DATA
#define CFG_FIFO_SIZE 			1
#define DMA_MAX_BURST_LENGTH 	BURST_LENGTH
#define DMA_MAX_BURST_NUM		5
// #define DATA_FIFO_SIZE 			BURST_LENGTH*3 // burst length * 3
#define MIN(a,b) 				((a < b ? a : ((b == 0)? 1 : b )))
#define MAX(a,b) 				(a > b ? a : b)

#define IFMAP_DDR_ADDR			0
#define FMAP_DDR_ADDR			  20300	
#define	OFMAP_DDR_ADDR			40300
	
#define IFMAP_GLB_ADDR			0
#define FMAP_GLB_ADDR			  0
#define	OFMAP_GLB_ADDR			0

//////////////////////////////////////////////////////////////////////////
///////////////////////////BUS CONFIG/////////////////////////////////////
#define NUM_OF_MASTER			3
#define NUM_OF_SLAVE			1


//////////////////////////////////////////////////////////////////////////
///////////////////////////TIME INFORMATION////////////////////////////////
#define DMA_SET_GO_LATENCY		80
#define BUSY_CHECK_PREIOD 		16

#define AW2WREADY_LATENCY		4

#define DDR_FW_PATH_LATENCY		5

#define DDR_R_BW_PATH_LATENCY	7
#define DDR_W_BW_PATH_LATENCY	1

#define MULTIPLE_OUT_LATENCY	3

#define READ_ARBIT_LATENCY		4					//need to debug, only run when it is 4
#define WRITE_ARBIT_LATENCY		5					

#define XBAR_DMA_DATA_PIPLINE	2

#define RDMA_SIF_DATA_PIPLINE	4 		//need to check which one is correct BL or 4 cycles
#define WDMA_SIF_DATA_PIPLINE	10

#define PROCESSING_PASS_DELAY	((dp_TE*dp_TF*lp_R*lp_S)+1)


//////////////////////////////////////////////////////////////////////////
//////////DDR CONFIG//////////////////////////////////////////////////////
#define MEMSIZE       10000
//#define WRITE_DONE      (((lp_M/dp_TM)*(lp_B/dp_TB)) * (dp_TB*dp_TM*dp_TE*(ceil((float)((float)(dp_TF)/(float)(BURST_LENGTH))))) * (ceil((float)((float)(BURST_LENGTH)/(float)(8)))))
#define DEVICE_INI      "ini/DDR3_micron_64M_8B_x4_sg15.ini"
#define DRMASIM2_ROOT     "../simplebus/dramsim2"
#define PL_CLK_FREQ    50000000          // Hz
#define PL_CLK_PRD            (PL_CLK_FREQ*0.0000004)

#define DATA_FIFO_SIZE      BURST_LENGTH*10 // burst length * 2

//////////////////////////////////////////////////////////////////////////
//////////LOG for DEBUG///////////////////////////////////////////////////
#define CYCLES(x)       PL_CLK_PRD*x, sc_core::SC_NS
// #define SC_SHOW_TIME(msg) {std::cout << "time " << setw(10)<< sc_time_stamp() << ": "<< setw(15) << name() << setw(25) << __func__ << setw(25) << /*__LINE__ << " line : " <<*/  msg << std::endl; }
// if (sc_time_stamp().to_seconds() > 0.018451)		
// if (sc_time_stamp() > sc_core::sc_time(18451,SC_US) )
#define SC_SHOW_TIME(msg) if(sc_time_stamp() > sc_time(1,SC_US) ) {std::cout << "time " << setw(10)<< sc_time_stamp() << ": "<< setw(15) << name() << setw(16) << __func__ <</* setw(25) <<*/ /*__LINE__ << " line : " <<*/  msg << std::endl;}

#define DATA_TRANSFER_ON

//#define HANDSHAKE_PROTOCOL

// #define ACC_CTRL_STATUS_LOG
//#define ACC_LOG
//#define ACC_PHASE_LOG
//#define ACC_STATU_LO
//#define DDR_LOG
//#define DMA_LOG
// #define Controller_LOG

//#define BUS_LOG

#ifdef BUS_LOG
  #define BUS_LOG_LEVEL 2
  // #define BUS_PHASE_LOG 	//=> BUS_LOG_LEVEL 1
  // #define BUS_STATUS_LOG	//=> BUS_LOG_LEVEL 2
#else
  #define BUS_LOG_LEVEL 0
#endif

#ifdef DMA_LOG
  #define DMA_LOG_LEVEL 1
#else
  #define DMA_LOG_LEVEL 0
#endif

#ifdef DDR_LOG
  #define DDR_LOG_LEVEL 1
#else
  #define DDR_LOG_LEVEL 0
#endif

#ifdef HANDSHAKE_PROTOCOL
  #define HANDSHAKE_LOG_LEVEL 1
#else
  #define HANDSHAKE_LOG_LEVEL 0
#endif

#ifndef HANDSHAKE_LOG_LEVEL
  HANDSHAKE_LOG_LEVEL 0
#endif

#define HANDSHAKE_PROTOCOL(log_lv,msg) \
  { \
    if (HANDSHAKE_LOG_LEVEL==(log_lv)) \
      SC_SHOW_TIME(msg) \
  }
