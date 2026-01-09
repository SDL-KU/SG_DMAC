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
2019.06.30: SoC Design Laboratory, Konkuk University.
*******************************************************************************/

#include <systemc>
#include "DSE.h"
#include "LOG.h"
#include "PROBE.h"
#include <iomanip>
// GreenControl + GreenConfig
#include <time.h>
#include "greencontrol/config.h"


#include <fstream>
#include <bitset>

#ifdef VCD
ofstream fout_vcd;
#endif

#include "simplebus_LSH_DMA_PAPER/acc/uProcessor.h"
#ifdef SDL_BUS
  #include "simplebus_LSH_DMA_PAPER/sdl_axi_interconnect/sdl_axi_interconnect.h"
#else
  #include "simplebus_LSH_DMA_PAPER/axibus.h"
#endif
#ifdef SG_MODE
  #include "simplebus_LSH_DMA_PAPER/TOP_ACC_NLR/DMA/rSGDMA.h"
  #include "simplebus_LSH_DMA_PAPER/TOP_ACC_NLR/DMA/rSGDMA_reg_polling.h"
  #include "simplebus_LSH_DMA_PAPER/TOP_ACC_NLR/DMA/rOPT_pSGDMA.h"
  #include "simplebus_LSH_DMA_PAPER/TOP_ACC_NLR/DMA/rOPT_pSGDMA_v2.h"
  #include "simplebus_LSH_DMA_PAPER/TOP_ACC_NLR/DMA/rOPT_pSGDMA_v3.h"
  #include "simplebus_LSH_DMA_PAPER/TOP_ACC_NLR/DMA/wSGDMA.h"
  #include "simplebus_LSH_DMA_PAPER/TOP_ACC_NLR/DMA/wOPT_pSGDMA.h"
  #include "simplebus_LSH_DMA_PAPER/TOP_ACC_NLR/DMA/wOPT_pSGDMA_v3.h"
#elif defined UDSG_MODE
  #include "simplebus_LSH_DMA_PAPER/TOP_ACC_NLR/DMA/rDMA_udsg.h"
  #include "simplebus_LSH_DMA_PAPER/TOP_ACC_NLR/DMA/wDMA_udsg.h"
#else
  #include "simplebus_LSH_DMA_PAPER/TOP_ACC_NLR/DMA/rDMA.h"
  #include "simplebus_LSH_DMA_PAPER/TOP_ACC_NLR/DMA/wDMA.h"
#endif
#include "simplebus_LSH_DMA_PAPER/TOP_ACC_NLR/top_acc_rev1.h"

#include "simplebus_LSH_DMA_PAPER/dramsim2_tlm_wrapper.h"
#include "simplebus_LSH_DMA_PAPER/dramsim2_tlm.h"

#include "simplebus_LSH_DMA_PAPER/beholder.h"


ofstream fout_ACC_POWER;
ofstream fout_DRAM_POWER;
ofstream fout_MACPS;
ofstream fout_Time;
ofstream fout_DRutil;
ofstream fout_TItv0;
ofstream fout_TItv1;
ofstream fout_TItv2;
ofstream fout_sweep;
ofstream fout_Etc0;
ofstream fout_Etc1;
ofstream fout_Etc2;

////////////////////////////////////////
// main
////////////////////////////////////////

int sc_main(int, char**)
{
  /// GreenControl Core instance
  gs::ctr::GC_Core core;
  // GreenConfig Plugin
  gs::cnf::ConfigDatabase cnfdatabase("ConfigDatabase");
  gs::cnf::ConfigPlugin configPlugin(&cnfdatabase);

  sc_time clk_period = sc_time(PL_CLK_PRD, SC_NS);


  #ifdef SG_MODE
    #ifdef OPT_SG_V1
      rOPT_pSGDMA<unsigned char>* ifmap_RDMA;
      rOPT_pSGDMA<unsigned char>* fmap_RDMA;
      wOPT_pSGDMA<unsigned char>* ofmap_WDMA;
    #elif defined OPT_SG_V2
      rOPT_pSGDMA_v2<unsigned char>* ifmap_RDMA;
      rOPT_pSGDMA_v2<unsigned char>* fmap_RDMA;
      wOPT_pSGDMA<unsigned char>* ofmap_WDMA;
    #elif defined OPT_SG_V3
      rOPT_pSGDMA_v3<unsigned char>* ifmap_RDMA;
      rOPT_pSGDMA_v3<unsigned char>* fmap_RDMA;
      wOPT_pSGDMA_v3<unsigned char>* ofmap_WDMA;
    #elif defined base_reg_polling
      rSGDMA_reg_polling<unsigned char>* ifmap_RDMA;
      rSGDMA_reg_polling<unsigned char>* fmap_RDMA;
      wSGDMA<unsigned char>* ofmap_WDMA;
    #else
      rSGDMA<unsigned char>* ifmap_RDMA;
      rSGDMA<unsigned char>* fmap_RDMA;
      wSGDMA<unsigned char>* ofmap_WDMA;
    #endif

  #elif defined UDSG_MODE
    rDMA_udsg<unsigned char>* ifmap_RDMA;
    rDMA_udsg<unsigned char>* fmap_RDMA;
    wDMA_udsg<unsigned char>* ofmap_WDMA;
  #else
    rDMA<unsigned char>* ifmap_RDMA;
    rDMA<unsigned char>* fmap_RDMA;
    wDMA<unsigned char>* ofmap_WDMA;
  #endif
  uProcessor*   ps;
  TOP_ACCELERATOR * top_acc;
  #ifdef SDL_BUS
    sdl_axi_interconnect *axi_xbar;
  #else
    axi_interconnect *axi_xbar;
  #endif

  DRAMSim2wrapper DRAMwrap("DRAMwrap",2);
  DRAMSim2_tlm DRAMSim2("DRAMSim2");
  DRAMwrap.target_port.base_addr=0x00000000;
  DRAMwrap.target_port.high_addr=0xffffffff;
  #ifdef SG_MODE
  #ifdef OPT_SG_V1
    ifmap_RDMA = new rOPT_pSGDMA<unsigned char>("ifmap_RDMA", DMA_MAX_BURST_LENGTH, DMA_MAX_BURST_NUM);
    fmap_RDMA = new rOPT_pSGDMA<unsigned char>("fmap_RDMA", DMA_MAX_BURST_LENGTH, DMA_MAX_BURST_NUM);
    ofmap_WDMA = new wOPT_pSGDMA<unsigned char>("ofmap_WDMA", DMA_MAX_BURST_LENGTH, DMA_MAX_BURST_NUM);
  #elif defined OPT_SG_V2
    ifmap_RDMA = new rOPT_pSGDMA_v2<unsigned char>("ifmap_RDMA", DMA_MAX_BURST_LENGTH, DMA_MAX_BURST_NUM);
    fmap_RDMA = new rOPT_pSGDMA_v2<unsigned char>("fmap_RDMA", DMA_MAX_BURST_LENGTH, DMA_MAX_BURST_NUM);
    ofmap_WDMA = new wOPT_pSGDMA<unsigned char>("ofmap_WDMA", DMA_MAX_BURST_LENGTH, DMA_MAX_BURST_NUM);
  #elif defined OPT_SG_V3
    ifmap_RDMA = new rOPT_pSGDMA_v3<unsigned char>("ifmap_RDMA", DMA_MAX_BURST_LENGTH, DMA_MAX_BURST_NUM);
    fmap_RDMA = new rOPT_pSGDMA_v3<unsigned char>("fmap_RDMA", DMA_MAX_BURST_LENGTH, DMA_MAX_BURST_NUM);
    ofmap_WDMA = new wOPT_pSGDMA_v3<unsigned char>("ofmap_WDMA", DMA_MAX_BURST_LENGTH, DMA_MAX_BURST_NUM);
  #elif defined base_reg_polling
    ifmap_RDMA = new rSGDMA_reg_polling<unsigned char>("ifmap_RDMA", DMA_MAX_BURST_LENGTH, DMA_MAX_BURST_NUM);
    fmap_RDMA = new rSGDMA_reg_polling<unsigned char>("fmap_RDMA", DMA_MAX_BURST_LENGTH, DMA_MAX_BURST_NUM);
    ofmap_WDMA = new wSGDMA<unsigned char>("ofmap_WDMA", DMA_MAX_BURST_LENGTH, DMA_MAX_BURST_NUM);
  #else
    ifmap_RDMA = new rSGDMA<unsigned char>("ifmap_RDMA", DMA_MAX_BURST_LENGTH, DMA_MAX_BURST_NUM);
    fmap_RDMA = new rSGDMA<unsigned char>("fmap_RDMA", DMA_MAX_BURST_LENGTH, DMA_MAX_BURST_NUM);
    ofmap_WDMA = new wSGDMA<unsigned char>("ofmap_WDMA", DMA_MAX_BURST_LENGTH, DMA_MAX_BURST_NUM);
  #endif
  #elif defined UDSG_MODE
    ifmap_RDMA = new rDMA_udsg<unsigned char>("ifmap_RDMA", DMA_MAX_BURST_LENGTH, DMA_MAX_BURST_NUM);
    fmap_RDMA = new rDMA_udsg<unsigned char>("fmap_RDMA", DMA_MAX_BURST_LENGTH, DMA_MAX_BURST_NUM);
    ofmap_WDMA = new wDMA_udsg<unsigned char>("ofmap_WDMA", DMA_MAX_BURST_LENGTH, DMA_MAX_BURST_NUM);
  #else
    ifmap_RDMA = new rDMA<unsigned char>("ifmap_RDMA", DMA_MAX_BURST_LENGTH, DMA_MAX_BURST_NUM);
    fmap_RDMA = new rDMA<unsigned char>("fmap_RDMA", DMA_MAX_BURST_LENGTH, DMA_MAX_BURST_NUM);
    ofmap_WDMA = new wDMA<unsigned char>("ofmap_WDMA", DMA_MAX_BURST_LENGTH, DMA_MAX_BURST_NUM);
  #endif
  top_acc = new TOP_ACCELERATOR("TOP_ACC", clk_period);
  ps = new uProcessor("ps");
  #ifdef SDL_BUS
    axi_xbar = new sdl_axi_interconnect("axi_xbar", NUM_OF_MASTER, NUM_OF_SLAVE);
  #else
    axi_xbar = new axi_interconnect("axi_xbar", NUM_OF_MASTER);
  #endif

  ps->uP2Acc.bind(*(top_acc->ACC_CTRL_tc_socket));
  ps->uP2rdma_ia.bind(ifmap_RDMA->uP2DMA);
  ps->uP2rdma_w.bind(fmap_RDMA->uP2DMA);
  ps->uP2wdma_oa.bind(ofmap_WDMA->uP2DMA); 
  ps->uP2ddr.bind(DRAMwrap.uP2ddr);

  ifmap_RDMA->t_slave.bind(*(top_acc->ifmap_SIF_L_t));
  fmap_RDMA->t_slave.bind(*(top_acc->fmap_SIF_L_t));

  top_acc->ofmap_SIF_S_i->bind(ofmap_WDMA->t_master);

  ifmap_RDMA->m_axi_mm2s(axi_xbar->target_port);
  fmap_RDMA->m_axi_mm2s(axi_xbar->target_port);    
  ofmap_WDMA->m_axi_s2mm(axi_xbar->target_port);
  #ifdef SG_MODE
    ifmap_RDMA->m_axi_sg(axi_xbar->target_port);
    fmap_RDMA->m_axi_sg(axi_xbar->target_port);
    ofmap_WDMA->m_axi_sg(axi_xbar->target_port);
  #endif

  axi_xbar->init_port(DRAMwrap.target_port);
  
  DRAMwrap.dramsim_port(DRAMSim2);
  DRAMSim2.dramsim_wrapper_target(DRAMwrap);

  beholder beholder0("beholder0");

  // beholder0.ptr_roi_pass_rep_start = reinterpret_cast<sc_event*>(behold_roi_pass_rep_start);
  beholder0.ptr_roi_pass_rep_start = &behold_roi_pass_rep_start;
//  beholder0.ptr_dma_done[0] = &(ifmap_RDMA->scp_done_axi_dat);
//  beholder0.ptr_dma_done[1] = &( fmap_RDMA->scp_done_axi_dat);
//  beholder0.ptr_dma_done[2] = &(ofmap_WDMA->scp_done_axi_dat);
//  beholder0.ptr_dma_trdy[0] = &(ifmap_RDMA->scp_tready);
//  beholder0.ptr_dma_trdy[1] = &( fmap_RDMA->scp_tready);
  // beholder0.ptr_acc_compute = top_acc->scp_ptr_acc_compute;


  float fSimlationCPUTime=0;
  float fSysCTimeStart=0;
  float fSysCTimeStop = 0;

  // ps->dbg_ptr_top_acc = top_acc;

  #ifdef VCD
  struct tm *t;
  time_t timer;
  timer = time(NULL);
  t = localtime(&timer);
  fout_vcd.open("log/vcd.vcd",std::ios_base::out | std::ios_base::app);
  fout_vcd<< "$date" <<endl;
  fout_vcd<< t->tm_year+1900 << "-" << t->tm_mon+1<< "-" << t->tm_mday <<endl;
  fout_vcd<< "$end" <<endl;
  fout_vcd<< "$version" <<endl;
  fout_vcd<< "SDL Simulator v1.4." <<endl;
  fout_vcd<< "$end" <<endl;
  fout_vcd<< "$comment" <<endl;
  fout_vcd<< "SoC Design Laboratory, Konkuk University." <<endl;
  fout_vcd<< "$end" <<endl;
  fout_vcd<< "$timescale 1ns $end" <<endl;
  fout_vcd<< "$scope module M_AXI0_SG $end" <<endl;
  fout_vcd<< "$var wire 32 #aa RDATA $end" <<endl;
  fout_vcd<< "$var wire 1 $aa ARVALID $end" <<endl;
  fout_vcd<< "$var wire 1 %aa ARREADY $end" <<endl;
  fout_vcd<< "$var wire 1 &aa RVALID $end" <<endl;
  fout_vcd<< "$var wire 1 'aa RREADY $end" <<endl;
  fout_vcd<< "$var wire 1 (aa RLAST $end" <<endl;
  fout_vcd<< "$var wire 32 )aa ARADDR $end" <<endl;
  fout_vcd<< "$var wire 8 *aa ARLEN $end" <<endl;
  fout_vcd<< "$var wire 32 ##aa WDATA $end" <<endl;
  fout_vcd<< "$var wire 1 $$aa AWVALID $end" <<endl;
  fout_vcd<< "$var wire 1 %%aa AWREADY $end" <<endl;
  fout_vcd<< "$var wire 1 &&aa WVALID $end" <<endl;
  fout_vcd<< "$var wire 1 ''aa WREADY $end" <<endl;
  fout_vcd<< "$var wire 1 ((aa WLAST $end" <<endl;
  fout_vcd<< "$var wire 32 ))aa AWADDR $end" <<endl;
  fout_vcd<< "$var wire 8 **aa AWLEN $end" <<endl;
  fout_vcd<< "$var wire 1 ~aa BVALID $end" <<endl;
  fout_vcd<< "$var wire 1 ~~aa BREADY $end" <<endl;
  fout_vcd<< "$upscope $end" <<endl;

  fout_vcd<< "$scope module M_AXI0_MM2S $end" <<endl;
  fout_vcd<< "$var wire 32 #a RDATA $end" <<endl;
  fout_vcd<< "$var wire 1 $a ARVALID $end" <<endl;
  fout_vcd<< "$var wire 1 %a ARREADY $end" <<endl;
  fout_vcd<< "$var wire 1 &a RVALID $end" <<endl;
  fout_vcd<< "$var wire 1 'a RREADY $end" <<endl;
  fout_vcd<< "$var wire 1 (a RLAST $end" <<endl;
  fout_vcd<< "$var wire 32 )a ARADDR $end" <<endl;
  fout_vcd<< "$var wire 8 *a ARLEN $end" <<endl;
  fout_vcd<< "$upscope $end" <<endl;

  fout_vcd<< "$scope module M_AXI1_SG $end" <<endl;
  fout_vcd<< "$var wire 32 #bb RDATA $end" <<endl;
  fout_vcd<< "$var wire 1 $bb ARVALID $end" <<endl;
  fout_vcd<< "$var wire 1 %bb ARREADY $end" <<endl;
  fout_vcd<< "$var wire 1 &bb RVALID $end" <<endl;
  fout_vcd<< "$var wire 1 'bb RREADY $end" <<endl;
  fout_vcd<< "$var wire 1 (bb RLAST $end" <<endl;
  fout_vcd<< "$var wire 32 )bb ARADDR $end" <<endl;
  fout_vcd<< "$var wire 8 *bb ARLEN $end" <<endl;
  fout_vcd<< "$var wire 32 ##bb WDATA $end" <<endl;
  fout_vcd<< "$var wire 1 $$bb AWVALID $end" <<endl;
  fout_vcd<< "$var wire 1 %%bb AWREADY $end" <<endl;
  fout_vcd<< "$var wire 1 &&bb WVALID $end" <<endl;
  fout_vcd<< "$var wire 1 ''bb WREADY $end" <<endl;
  fout_vcd<< "$var wire 1 ((bb WLAST $end" <<endl;
  fout_vcd<< "$var wire 32 ))bb AWADDR $end" <<endl;
  fout_vcd<< "$var wire 8 **bb AWLEN $end" <<endl;
  fout_vcd<< "$var wire 1 ~bb BVALID $end" <<endl;
  fout_vcd<< "$var wire 1 ~~bb BREADY $end" <<endl;
  fout_vcd<< "$upscope $end" <<endl;

  fout_vcd<< "$scope module M_AXI1_MM2S $end" <<endl;
  fout_vcd<< "$var wire 32 #b RDATA $end" <<endl;
  fout_vcd<< "$var wire 1 $b ARVALID $end" <<endl;
  fout_vcd<< "$var wire 1 %b ARREADY $end" <<endl;
  fout_vcd<< "$var wire 1 &b RVALID $end" <<endl;
  fout_vcd<< "$var wire 1 'b RREADY $end" <<endl;
  fout_vcd<< "$var wire 1 (b RLAST $end" <<endl;
  fout_vcd<< "$var wire 32 )b ARADDR $end" <<endl;
  fout_vcd<< "$var wire 8 *b ARLEN $end" <<endl;
  fout_vcd<< "$upscope $end" <<endl;

  fout_vcd<< "$scope module M_AXI2_SG $end" <<endl;
  fout_vcd<< "$var wire 32 #cc RDATA $end" <<endl;
  fout_vcd<< "$var wire 1 $cc ARVALID $end" <<endl;
  fout_vcd<< "$var wire 1 %cc ARREADY $end" <<endl;
  fout_vcd<< "$var wire 1 &cc RVALID $end" <<endl;
  fout_vcd<< "$var wire 1 'cc RREADY $end" <<endl;
  fout_vcd<< "$var wire 1 (cc RLAST $end" <<endl;
  fout_vcd<< "$var wire 32 )cc ARADDR $end" <<endl;
  fout_vcd<< "$var wire 8 *cc ARLEN $end" <<endl;
  fout_vcd<< "$var wire 32 ##cc WDATA $end" <<endl;
  fout_vcd<< "$var wire 1 $$cc AWVALID $end" <<endl;
  fout_vcd<< "$var wire 1 %%cc AWREADY $end" <<endl;
  fout_vcd<< "$var wire 1 &&cc WVALID $end" <<endl;
  fout_vcd<< "$var wire 1 ''cc WREADY $end" <<endl;
  fout_vcd<< "$var wire 1 ((cc WLAST $end" <<endl;
  fout_vcd<< "$var wire 32 ))cc AWADDR $end" <<endl;
  fout_vcd<< "$var wire 8 **cc AWLEN $end" <<endl;
  fout_vcd<< "$var wire 1 ~cc BVALID $end" <<endl;
  fout_vcd<< "$var wire 1 ~~cc BREADY $end" <<endl;
  fout_vcd<< "$upscope $end" <<endl;

  fout_vcd<< "$scope module M_AXI2_S2MM $end" <<endl;
  fout_vcd<< "$var wire 32 ## WDATA $end" <<endl;
  fout_vcd<< "$var wire 1 $$ AWVALID $end" <<endl;
  fout_vcd<< "$var wire 1 %% AWREADY $end" <<endl;
  fout_vcd<< "$var wire 1 && WVALID $end" <<endl;
  fout_vcd<< "$var wire 1 '' WREADY $end" <<endl;
  fout_vcd<< "$var wire 1 (( WLAST $end" <<endl;
  fout_vcd<< "$var wire 32 )) AWADDR $end" <<endl;
  fout_vcd<< "$var wire 8 ** AWLEN $end" <<endl;
  fout_vcd<< "$var wire 1 ~ BVALID $end" <<endl;
  fout_vcd<< "$var wire 1 ~~ BREADY $end" <<endl;
  fout_vcd<< "$upscope $end" <<endl;
  fout_vcd<< "$scope module Computation $end" <<endl;
  fout_vcd<< "$var wire 1 ^^ Computation $end" <<endl;
  fout_vcd<< "$upscope $end" <<endl;

  fout_vcd<< "$scope module S_AXI_HP0 $end" <<endl;
  fout_vcd<< "$var wire 32 #@ RDATA $end" <<endl;
  fout_vcd<< "$var wire 1 $@ ARVALID $end" <<endl;
  fout_vcd<< "$var wire 1 %@ ARREADY $end" <<endl;
  fout_vcd<< "$var wire 1 &@ RVALID $end" <<endl;
  fout_vcd<< "$var wire 1 '@ RREADY $end" <<endl;
  fout_vcd<< "$var wire 1 (@ RLAST $end" <<endl;
  fout_vcd<< "$var wire 32 )@ ARADDR $end" <<endl;
  fout_vcd<< "$var wire 8 *@ ARLEN $end" <<endl;
  fout_vcd<< "$var wire 32 ##@ WDATA $end" <<endl;
  fout_vcd<< "$var wire 1 $$@ AWVALID $end" <<endl;
  fout_vcd<< "$var wire 1 %%@ AWREADY $end" <<endl;
  fout_vcd<< "$var wire 1 &&@ WVALID $end" <<endl;
  fout_vcd<< "$var wire 1 ''@ WREADY $end" <<endl;
  fout_vcd<< "$var wire 1 ((@ WLAST $end" <<endl;
  fout_vcd<< "$var wire 32 ))@ AWADDR $end" <<endl;
  fout_vcd<< "$var wire 8 **@ AWLEN $end" <<endl;
  fout_vcd<< "$var wire 1 ~@ BVALID $end" <<endl;
  fout_vcd<< "$var wire 1 ~~@ BREADY $end" <<endl;
  fout_vcd<< "$upscope $end" <<endl;

  fout_vcd<< "$scope module DRAMSim2 $end" <<endl;
  fout_vcd<< "$scope module TransQueue $end" <<endl;
  for(int i = 0; i<32; i++)
    fout_vcd<< "$var wire 3 TQT" << i << " TransQueue" << i << " $end" <<endl;
  fout_vcd<< "$upscope $end" <<endl;
  fout_vcd<< "$scope module CommandQueue $end" <<endl;
  for(int i = 0; i<32; i++)
    fout_vcd<< "$var wire 20 CQA" << i << " CommandQueue" << i << " $end" <<endl;
  fout_vcd<< "$var wire 8 NOCQA" << " NoCommand" << " $end" <<endl;
  fout_vcd<< "$upscope $end" <<endl;
  fout_vcd<< "$scope module BANK[8] $end" <<endl;
  for(int i = 0; i<8; i++)
    fout_vcd<< "$var wire 27 BANK" << i << " BANKState" << i << " $end" <<endl;
  fout_vcd<< "$upscope $end" <<endl;
  fout_vcd<< "$upscope $end" <<endl;


  fout_vcd<< "$enddefinitions $end" <<endl;
  fout_vcd<< "$dumpvars" <<endl;  
  fout_vcd<< "0$aa" <<endl;
  fout_vcd<< "0&aa"  <<endl;
  fout_vcd<< "0&&aa" <<endl;
  fout_vcd<< "0$$aa" <<endl;
  fout_vcd<< "0''aa" <<endl;
  fout_vcd<< "0~aa"  <<endl;
  fout_vcd<< "bxxxxxxxx #a" <<endl;
  fout_vcd<< "0$a" <<endl;
  fout_vcd<< "0%a" <<endl;
  fout_vcd<< "0&a" <<endl;
  fout_vcd<< "0'a" <<endl;
  fout_vcd<< "0(a" <<endl;
  fout_vcd<< "bxxxxxxxx )a" <<endl;
  fout_vcd<< "bxxxxxxxx *a" <<endl;
  fout_vcd<< "0$bb" <<endl;
  fout_vcd<< "0&bb"  <<endl;
  fout_vcd<< "0&&bb" <<endl;
  fout_vcd<< "0$$bb" <<endl;
  fout_vcd<< "0''bb" <<endl;
  fout_vcd<< "0~bb"  <<endl;
  fout_vcd<< "bxxxxxxxx #b" <<endl;
  fout_vcd<< "0$b" <<endl;
  fout_vcd<< "0%b" <<endl;
  fout_vcd<< "0&b" <<endl;
  fout_vcd<< "0'b" <<endl;
  fout_vcd<< "0(b" <<endl;
  fout_vcd<< "bxxxxxxxx )b" <<endl;
  fout_vcd<< "bxxxxxxxx *b" <<endl;
  fout_vcd<< "bxxxxxxxx ##" <<endl;
  fout_vcd<< "0$$" <<endl;
  fout_vcd<< "0%%" <<endl;
  fout_vcd<< "0&&" <<endl;
  fout_vcd<< "0''" <<endl;
  fout_vcd<< "0((" <<endl;
  fout_vcd<< "bxxxxxxxx ))" <<endl;
  fout_vcd<< "bxxxxxxxx **" <<endl;
  fout_vcd<< "0~" <<endl;
  fout_vcd<< "0~~" <<endl;
  fout_vcd<< "bxxxxxxxx #@" <<endl;
  fout_vcd<< "0$@" <<endl;
  fout_vcd<< "0%@" <<endl;
  fout_vcd<< "0&@" <<endl;
  fout_vcd<< "0'@" <<endl;
  fout_vcd<< "0(@" <<endl;
  fout_vcd<< "bxxxxxxxx )@" <<endl;
  fout_vcd<< "bxxxxxxxx *@" <<endl;
  fout_vcd<< "bxxxxxxxx ##@" <<endl;
  fout_vcd<< "0$$@" <<endl;
  fout_vcd<< "0%%@" <<endl;
  fout_vcd<< "0&&@" <<endl;
  fout_vcd<< "0''@" <<endl;
  fout_vcd<< "0((@" <<endl;
  fout_vcd<< "bxxxxxxxx ))@" <<endl;
  fout_vcd<< "bxxxxxxxx **@" <<endl;
  fout_vcd<< "0~@" <<endl;
  fout_vcd<< "0~~@" <<endl;

  fout_vcd<< "0$cc" <<endl;
  fout_vcd<< "0&cc"  <<endl;
  fout_vcd<< "0&&cc" <<endl;
  fout_vcd<< "0$$cc" <<endl;
  fout_vcd<< "0''cc" <<endl;
  fout_vcd<< "0~cc"  <<endl;

  for(int i = 0; i<32; i++)
    fout_vcd<< "bxxx TQT" << i << endl;
  for(int i = 0; i<32; i++)
    fout_vcd<< "bxxxxxxxx CQA" << i << endl;
  for(int i = 0; i<8; i++)
    fout_vcd<< "b11000000000000000000000 BANK" << i << endl;
  
  fout_vcd<< "$end" <<endl;
  #endif

#ifdef SIM_RESULT
    // cout << endl << "Simualtion Results: "<< endl;
    // cout << endl << "*AXI Xbar utilization -->" << endl;
#endif

  fSysCTimeStart = (float)sc_time_stamp().to_seconds();
  clock();
  sc_core::sc_start();

  fSysCTimeStop = (float)sc_time_stamp().to_seconds();
  fSimlationCPUTime = (float)((float)clock() / (float)CLOCKS_PER_SEC);

  int total_cycle = (fSysCTimeStop - fSysCTimeStart) * PL_CLK_FREQ;
  
#ifdef SIM_RESULT
  cout<<"*DRAM (Bank Information) --> Check the 'DRAM_BankState.log' for more information" << endl;
 
  // cout<<"(Bank0) ACTIVATION: " << B0_ACTV << endl;
  // cout<<"(Bank0) PRECHARGE : " << B0_PREC << endl;
  // cout<<"(Bank0) IDLE      : " << B0_IDLE << endl; 
  // cout<<"(Bank0) PowerDown : " << B0_LOWP << endl; cout << endl;

  // cout<<"(Bank1) ACTIVATION: " << B1_ACTV << endl;
  // cout<<"(Bank1) PRECHARGE : " << B1_PREC << endl;
  // cout<<"(Bank1) IDLE      : " << B1_IDLE << endl; 
  // cout<<"(Bank1) PowerDown : " << B1_LOWP << endl; cout << endl;

  // cout<<"(Bank2) ACTIVATION: " << B2_ACTV << endl;
  // cout<<"(Bank2) PRECHARGE : " << B2_PREC << endl;
  // cout<<"(Bank2) IDLE      : " << B2_IDLE << endl; 
  // cout<<"(Bank2) PowerDown : " << B2_LOWP << endl; cout << endl;

  // cout<<"(Bank3) ACTIVATION: " << B3_ACTV << endl;
  // cout<<"(Bank3) PRECHARGE : " << B3_PREC << endl;
  // cout<<"(Bank3) IDLE      : " << B3_IDLE << endl; 
  // cout<<"(Bank3) PowerDown : " << B3_LOWP << endl; cout << endl;

  // cout<<"(Bank4) ACTIVATION: " << B4_ACTV << endl;
  // cout<<"(Bank4) PRECHARGE : " << B4_PREC << endl;
  // cout<<"(Bank4) IDLE      : " << B4_IDLE << endl; 
  // cout<<"(Bank4) PowerDown : " << B4_LOWP << endl; cout << endl;

  // cout<<"(Bank5) ACTIVATION: " << B5_ACTV << endl;
  // cout<<"(Bank5) PRECHARGE : " << B5_PREC << endl;
  // cout<<"(Bank5) IDLE      : " << B5_IDLE << endl; 
  // cout<<"(Bank5) PowerDown : " << B5_LOWP << endl; cout <<  endl;

  // cout<<"(Bank6) ACTIVATION: " << B6_ACTV << endl;
  // cout<<"(Bank6) PRECHARGE : " << B6_PREC << endl;
  // cout<<"(Bank6) IDLE      : " << B6_IDLE << endl; 
  // cout<<"(Bank6) PowerDown : " << B6_LOWP << endl; cout <<  endl;

  // cout<<"(Bank7) ACTIVATION: " << B7_ACTV << endl;
  // cout<<"(Bank7) PRECHARGE : " << B7_PREC << endl;
  // cout<<"(Bank7) IDLE      : " << B7_IDLE << endl; 
  // cout<<"(Bank7) PowerDown : " << B7_LOWP << endl; cout << endl;

  // cout<<"(Device) REFRESH           : " << B0_REFH << endl; 
  
  // cout<<"(Device) DRAM Cycles       : " << DRAMCycles << endl;
  // cout<<"DRAM utilization           : " << (B0_ACTV+ B0_PREC) / DRAMCycles << endl;

  cout<<"(Bank0) ACTIVATION: " << B0_ACTV_CMD << endl;
  cout<<"(Bank0) PRECHARGE : " << B0_PREC_CMD << endl;
  cout<<"(Bank0) READ      : " << B0_READ_CMD << endl; 
  cout<<"(Bank0) WRITE     : " << B0_WRIT_CMD << endl; 
  cout<<"(Bank0) REFRESH   : " << B0_REFH_CMD << endl; cout << endl;

  cout<<"(Bank1) ACTIVATION: " << B1_ACTV_CMD << endl;
  cout<<"(Bank1) PRECHARGE : " << B1_PREC_CMD << endl;
  cout<<"(Bank1) READ      : " << B1_READ_CMD << endl; 
  cout<<"(Bank1) WRITE     : " << B1_WRIT_CMD << endl; 
  cout<<"(Bank1) REFRESH   : " << B1_REFH_CMD << endl; cout << endl;

  cout<<"(Bank2) ACTIVATION: " << B2_ACTV_CMD << endl;
  cout<<"(Bank2) PRECHARGE : " << B2_PREC_CMD << endl;
  cout<<"(Bank2) READ      : " << B2_READ_CMD << endl; 
  cout<<"(Bank2) WRITE     : " << B2_WRIT_CMD << endl; 
  cout<<"(Bank2) REFRESH   : " << B2_REFH_CMD << endl; cout << endl;

  cout<<"(Bank3) ACTIVATION: " << B3_ACTV_CMD << endl;
  cout<<"(Bank3) PRECHARGE : " << B3_PREC_CMD << endl;
  cout<<"(Bank3) READ      : " << B3_READ_CMD << endl; 
  cout<<"(Bank3) WRITE     : " << B3_WRIT_CMD << endl; 
  cout<<"(Bank3) REFRESH   : " << B3_REFH_CMD << endl; cout << endl;

  cout<<"(Bank4) ACTIVATION: " << B4_ACTV_CMD << endl;
  cout<<"(Bank4) PRECHARGE : " << B4_PREC_CMD << endl;
  cout<<"(Bank4) READ      : " << B4_READ_CMD << endl; 
  cout<<"(Bank4) WRITE     : " << B4_WRIT_CMD << endl; 
  cout<<"(Bank4) REFRESH   : " << B4_REFH_CMD << endl; cout << endl;

  cout<<"(Bank5) ACTIVATION: " << B5_ACTV_CMD << endl;
  cout<<"(Bank5) PRECHARGE : " << B5_PREC_CMD << endl;
  cout<<"(Bank5) READ      : " << B5_READ_CMD << endl; 
  cout<<"(Bank5) WRITE     : " << B5_WRIT_CMD << endl; 
  cout<<"(Bank5) REFRESH   : " << B5_REFH_CMD << endl; cout << endl;

  cout<<"(Bank6) ACTIVATION: " << B6_ACTV_CMD << endl;
  cout<<"(Bank6) PRECHARGE : " << B6_PREC_CMD << endl;
  cout<<"(Bank6) READ      : " << B6_READ_CMD << endl; 
  cout<<"(Bank6) WRITE     : " << B6_WRIT_CMD << endl; 
  cout<<"(Bank6) REFRESH   : " << B6_REFH_CMD << endl; cout << endl;

  cout<<"(Bank7) ACTIVATION: " << B7_ACTV_CMD << endl;
  cout<<"(Bank7) PRECHARGE : " << B7_PREC_CMD << endl;
  cout<<"(Bank7) READ      : " << B7_READ_CMD << endl; 
  cout<<"(Bank7) WRITE     : " << B7_WRIT_CMD << endl; 
  cout<<"(Bank7) REFRESH   : " << B7_REFH_CMD << endl; cout << endl;


  cout<<endl<<"*Performance -->" << endl;
  cout<<"Execution  Time (msecs)        : "<< (fSysCTimeStop - fSysCTimeStart) * 1000 << endl;
  cout<<"MACs/Cycles     (Performance)  : " << ((float)TOTAL_MACs/total_cycle) << endl;  
#else
  // cout<<"Execution  Time (msecs)        : "<< (fSysCTimeStop - fSysCTimeStart) * 1000 << endl;
#endif

  // std::cout<<"               HOST time   : "<< fSimlationCPUTime << std::endl;
  // std::cout<<"       average pass time   : "<< fSimlationCPUTime/no_pass_executed << std::endl;
  // std::cout<<"          NUM_TOTAL_PASS   : "<< NUM_TOTAL_PASS << std::endl;
  // std::cout<<"    aprx total HOST time   : "
  //     << ((int)(fSimlationCPUTime/no_pass_executed*NUM_TOTAL_PASS)/86400) << "d "
  //     << ((int)(fSimlationCPUTime/no_pass_executed*NUM_TOTAL_PASS)/3600)%24   << "h " 
  //     << ((int)(fSimlationCPUTime/no_pass_executed*NUM_TOTAL_PASS)/60)%60    << "m "
  //     << ((int)(fSimlationCPUTime/no_pass_executed*NUM_TOTAL_PASS)%60)    << "s" << std::endl;
 
  fout_DRAM_POWER.open("log/dram_power.log",std::ios_base::out | std::ios_base::app);
  fout_DRAM_POWER << bgPower/3 << " " << ActPrePower/3 << " " << BurstPower/3 << " " << RefPower/3 << endl;
  fout_DRAM_POWER.close();

double cycle_cnt = (fSysCTimeInterval1/2);
double datawidth__ =16;
for(int i=0; i<dp_TC;i++){
  IA_COMM_WRITE[i] = dp_TH*dp_TW;
  IA_COMP_READ[i] = lp_R*lp_S*dp_TB*dp_TE*dp_TF;

  ia_sram_comm_power += SRAM_LEAKAGE + SRAM_DYNAMIC*IA_COMM_WRITE[i]/cycle_cnt;
  ia_sram_comp_power += SRAM_LEAKAGE + SRAM_DYNAMIC*IA_COMP_READ[i]/cycle_cnt;
}
for(int i=0; i<dp_TM*dp_TC;i++){
  WT_COMM_WRITE[i] = lp_R*lp_S;
  WT_COMP_READ[i] = lp_R*lp_S;

  wt_sram_comm_power += SRAM_LEAKAGE + SRAM_DYNAMIC*WT_COMM_WRITE[i]/cycle_cnt;
  wt_sram_comp_power += SRAM_LEAKAGE + SRAM_DYNAMIC*WT_COMP_READ[i]/cycle_cnt;
}
for(int i=0; i<dp_TM;i++){
  OA_COMM_READ[i] = ceil((dp_TB*dp_TM*dp_TE*dp_TF)/(lp_C/dp_TC));
  OA_COMM_WRITE[i] = 0;
  OA_COMP_READ[i] = lp_R*lp_S*dp_TB*dp_TE*dp_TF;
  OA_COMP_WRITE[i] = lp_R*lp_S*dp_TB*dp_TE*dp_TF;

  oa_sram_comm_power += SRAM_LEAKAGE 
                      + SRAM_DYNAMIC*OA_COMM_READ[i]/cycle_cnt
                      + SRAM_DYNAMIC*OA_COMM_WRITE[i]/cycle_cnt;
  oa_sram_comp_power += SRAM_LEAKAGE 
                      + SRAM_DYNAMIC*OA_COMP_READ[i]/cycle_cnt
                      + SRAM_DYNAMIC*OA_COMP_WRITE[i]/cycle_cnt;
}
for(int i=0; i<dp_TM*dp_TC;i++){
  MULTIPLIER[i] = lp_R*lp_S*dp_TB*dp_TE*dp_TF;
  ADDER[i] = lp_R*lp_S*dp_TB*dp_TE*dp_TF;

  multiplier_power += MULT_LEAKAGE + MULT_DYNAMIC*MULTIPLIER[i]/cycle_cnt;
  adder_power += ADD_LEAKAGE + ADD_DYNAMIC*ADDER[i]/cycle_cnt;
}
for(int i=0; i<(dp_TC+dp_TM*dp_TC+dp_TM)*2;i++){
  if(i<dp_TC)
    ACC_REGISTER[i] = lp_R*lp_S*dp_TB*dp_TE*dp_TF;
  else if(i<dp_TC+dp_TM*dp_TC)
    ACC_REGISTER[i] = lp_R*lp_S;
  else if(i<dp_TC+dp_TM*dp_TC+dp_TM)
    ACC_REGISTER[i] = lp_R*lp_S*dp_TB*dp_TE*dp_TF;
  else
    ACC_REGISTER[i] = 0;

  acc_register_power += REG_LEAKAGE + datawidth__*REG_DYNAMIC*ACC_REGISTER[i]/cycle_cnt;
}
double acc_overall_power =
                  ia_sram_comp_power +
                  ia_sram_comp_power +
                  wt_sram_comm_power +
                  wt_sram_comp_power +
                  oa_sram_comm_power +
                  oa_sram_comp_power +
                  multiplier_power +
                  adder_power +
                  acc_register_power;

cout<<cycle_cnt<<endl<<endl;
  fout_ACC_POWER.open("log/acc_power.log",std::ios_base::out | std::ios_base::app);
  cout << ia_sram_comm_power << " " << ia_sram_comp_power << " " << wt_sram_comm_power << " " << wt_sram_comp_power  << " " ;
  cout                << oa_sram_comm_power << " " << oa_sram_comp_power << " " << multiplier_power << " " << adder_power  << " " ;
  cout                << acc_register_power << " " <<  acc_overall_power << endl;
  fout_ACC_POWER.close();

delete[] IA_COMP_READ;
delete[] IA_COMM_WRITE;
delete[] WT_COMP_READ;
delete[] WT_COMM_WRITE;
delete[] OA_COMP_WRITE;
delete[] OA_COMP_READ;
delete[] OA_COMM_WRITE;
delete[] OA_COMM_READ;
delete[] MULTIPLIER;
delete[] ADDER;
delete[] ACC_REGISTER;





  fout_MACPS.open("log/macps.log",std::ios_base::out | std::ios_base::app);
  // fout_Time.open("log/extime.log",std::ios_base::out | std::ios_base::app);
  // fout_DRutil.open("log/drutil.log",std::ios_base::out | std::ios_base::app);
  fout_TItv0.open("log/titv0.log",std::ios_base::out | std::ios_base::app);
  fout_TItv1.open("log/titv1.log",std::ios_base::out | std::ios_base::app);
  fout_TItv2.open("log/titv2.log",std::ios_base::out | std::ios_base::app);
  fout_Etc0.open("log/etc0.log",std::ios_base::out | std::ios_base::app);
  fout_Etc1.open("log/etc1.log",std::ios_base::out | std::ios_base::app);
  fout_Etc2.open("log/etc2.log",std::ios_base::out | std::ios_base::app);

  // fout_MACPS<<((float)TOTAL_MACs/total_cycle) <<endl;
  fout_MACPS<<log_macps_rpass <<endl;
  // fout_Time<<(fSysCTimeStop - fSysCTimeStart) * 1000 <<endl;
  // fout_DRutil<<(B0_ACTV+ B0_PREC) / DRAMCycles << endl;
  fout_TItv0<< TimeInterval0 << endl;  
  fout_TItv1<< TimeInterval1 << endl;  
  fout_TItv2<< TimeInterval2 << endl;  
  fout_Etc0<< log_intv_a_dat[0]<<' '<<log_intv_a_dat[1]<<' '<<log_intv_a_dat[2]<< endl;
  fout_Etc1<< log_intv_b_dat[0]<<' '<<log_intv_b_dat[1]<<' '<<log_intv_b_dat[2]<< endl;
  fout_Etc2<< log_intv_c_dat[0]<<' '<<log_intv_c_dat[1]<<' '<<log_intv_c_dat[2]<< endl;

  fout_MACPS.close();
  // fout_Time.close();
  // fout_DRutil.close();
  fout_TItv0.close();
  fout_TItv1.close();
  fout_TItv2.close();
  fout_Etc0.close();
  fout_Etc1.close();
  fout_Etc2.close();
  #ifdef VCD
  fout_vcd.close();
  #endif

  delete ifmap_RDMA;
  delete fmap_RDMA;
  delete ofmap_WDMA;
  delete top_acc;
  delete ps;

  return 0;
}

int main(int argc, char** argv){
  
  if(argc ==10){
    PL_CLK_MHZ = atoi(argv[1]);
    dp_TM = atoi(argv[2]);
    dp_TC = atoi(argv[3]);
    dp_TB = atoi(argv[4]);
    dp_TE = atoi(argv[5]);
    dp_TF = atoi(argv[6]);
    dp_UM = dp_TM;
    dp_UC = dp_TC;
    DMA_MAX_BURST_LENGTH = 	atoi(argv[ 7]);
    DMA_MAX_BURST_NUM = 	atoi(argv[ 8]);
    MULTIPLE_OUT_LATENCY = 	atoi(argv[ 9]);
    
    cout <<"================================="<< endl;
    cout<< "CLK :" << PL_CLK_MHZ << "MHz"<<endl;
    cout<< "TM: " << dp_TM;
    cout<< ", TC: " << dp_TC;
    cout<< ", TB: " << dp_TB;
    cout<< ", TE: " << dp_TE;
    cout<< ", TF: " << dp_TF << endl;
    cout<< "BL: " << DMA_MAX_BURST_LENGTH;
    cout<< ", NMO: " << DMA_MAX_BURST_NUM;
    cout<< ", MOL: " << MULTIPLE_OUT_LATENCY << endl;

    DSE_param_dependency_sync();
  }
  else if(argc ==12){
    PL_CLK_MHZ = atoi(argv[1]);
    dp_TM = atoi(argv[2]);
    dp_TC = atoi(argv[3]);
    dp_TB = atoi(argv[4]);
    dp_TE = atoi(argv[5]);
    dp_TF = atoi(argv[6]);
    dp_UM = atoi(argv[7]);
    dp_UC = atoi(argv[8]);
    DMA_MAX_BURST_LENGTH =  atoi(argv[ 9]);
    DMA_MAX_BURST_NUM =   atoi(argv[10]);
    MULTIPLE_OUT_LATENCY =  atoi(argv[11]);
    
    cout <<"================================="<< endl;
    cout<< "CLK :" << PL_CLK_MHZ << "MHz"<<endl;
    cout<< "TM: " << dp_TM;
    cout<< ", TC: " << dp_TC;
    cout<< ", TB: " << dp_TB;
    cout<< ", TE: " << dp_TE;
    cout<< ", TF: " << dp_TF;
    cout<< ", UM: " << dp_UM;
    cout<< ", UC: " << dp_UC << endl;
    cout<< "BL: " << DMA_MAX_BURST_LENGTH;
    cout<< ", NMO: " << DMA_MAX_BURST_NUM;
    cout<< ", MOL: " << MULTIPLE_OUT_LATENCY << endl;

    DSE_param_dependency_sync();
  }
  else if(argc == 16 || argc == 18){
    //dma
    // sg_dma_dse_param.packet_length             = atoi(argv[ 1]);
    // sg_dma_dse_param.num_of_line_in_macroblock = atoi(argv[ 2]);
    // sg_dma_dse_param.num_of_macroblock         = atoi(argv[ 3]);
    // sg_dma_dse_param.num_of_packet             = sg_dma_dse_param.num_of_line_in_macroblock * sg_dma_dse_param.num_of_macroblock;
    
    dp_TM = atoi(argv[1]);
    sg_dma_dse_param.tc_for_dse = atoi(argv[2]);
    dp_TE = atoi(argv[3]);
    dp_TF = atoi(argv[4]);

    //bus
    READ_ARBIT_LATENCY                         = atoi(argv[ 5]);
    XBAR_DMA_DATA_PIPLINE                      = atoi(argv[ 6]);
    WRITE_ARBIT_LATENCY                        = atoi(argv[ 7]);
    //dma
    SG_RLAST_TO_NEXT_SG_ADDR_REQ_LATENCY       = atoi(argv[ 8]);
    TRANSFER_DONE_TO_NEXT_SG_ADDR_REQ_LATENCY  = SG_RLAST_TO_NEXT_SG_ADDR_REQ_LATENCY;
    MULTIPLE_OUT_LATENCY                       = atoi(argv[ 9])/2;
    SG_RLAST_TO_ADDR_REQ_LATENCY               = atoi(argv[ 9])/2;
    TRANSFER_DONE_TO_SG_AW_REQ_LATENCY         = atoi(argv[10]);
    AW2WREADY_LATENCY                          = atoi(argv[11]);
    SG_AW_HANDSHAKE_TO_W_REQ_LATENCY           = AW2WREADY_LATENCY;

    PL_CLK_MHZ                                 = atoi(argv[12]);
    sg_dma_dse_param.dse_option                = atoi(argv[13]);
    SG_R_NMOT                                  = atoi(argv[14]);
    DMA_MAX_BURST_NUM                          = atoi(argv[15]);

    if(sg_dma_dse_param.dse_option != 0 &&
        sg_dma_dse_param.dse_option != 1 &&
        sg_dma_dse_param.dse_option != 2 &&
        argc == 18){
      sg_dma_dse_param.packet_length             = atoi(argv[16]);
      sg_dma_dse_param.num_of_macroblock         = atoi(argv[17]);
      sg_dma_dse_param.num_of_line_in_macroblock = 1;
    }

    DSE_param_dependency_sync();

    cout <<"================================="<< endl;
    // cout << "SG DSE ";
    cout<< "CLK :" << PL_CLK_MHZ << "MHz ";
    cout<< ", TM: " << dp_TM;
    cout<< ", TC: " << sg_dma_dse_param.tc_for_dse;
    cout<< ", TE: " << dp_TE;
    cout<< ", TF: " << dp_TF;
    cout<< ", BL: " << DMA_MAX_BURST_LENGTH;
    cout<< ", SGNMO: " << SG_R_NMOT;
    cout<< ", MMNMO: " << DMA_MAX_BURST_NUM << endl;
  }
  else{
    cout <<"=================================" << endl;
    cout<< "No input, Simulate using default"<< endl;
  }

  sc_main(argc, argv);

  fout_sweep.open("log/sweep.log",std::ios_base::out | std::ios_base::app);
  fout_sweep<<PL_CLK_MHZ<<'\t';
  fout_sweep<<dp_TM<<'\t';
  fout_sweep<<dp_TC<<'\t';
  fout_sweep<<dp_TB<<'\t';
  fout_sweep<<dp_TE<<'\t';
  fout_sweep<<dp_TF<<'\t';
  fout_sweep<<DMA_MAX_BURST_LENGTH<<'\t';
  fout_sweep<<DMA_MAX_BURST_NUM<<'\t';
  fout_sweep<<MULTIPLE_OUT_LATENCY<<endl;
  fout_sweep.close();

  cout <<"================================="<< endl;

  return 0;
}
