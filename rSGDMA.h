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

// #pragma once
#include "systemc.h"
#include "tlm.h"

#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"
#include "tlm_utils/peq_with_cb_and_phase.h"

using namespace sc_core;
using namespace sc_dt;
using namespace std;

#ifndef DMA_LOG_LEVEL
DMA_LOG_LEVEL 0
#endif

#define DMA_LOG(log_lv, msg)         \
   {                                 \
      if (DMA_LOG_LEVEL >= (log_lv)) \
         SC_SHOW_TIME(msg)           \
   }

    struct rSGDMA_config
{
   unsigned int size;
   unsigned int src_addr, dst_addr;

   // it need when you want to push this struct to fifo
   friend ostream &operator<<(ostream &os, const rSGDMA_config &p)
   {
      os << "size is " << p.size << endl;
      os << "addr is " << p.src_addr << '~' << "dst_addr" << p.dst_addr << endl;
      return os;
   }

   void debug_print_data()
   {
      cout << "size : " << size << endl;
      cout << "src_addr : " << src_addr << endl;
      cout << "dst_addr : " << dst_addr << endl;
   }
};

struct SG_DMA_Register
{
   unsigned int mm2s_dmacr;
   unsigned int mm2s_dmasr;
   unsigned int mm2s_curdesc;
   unsigned int mm2s_curdesc_msb;
   unsigned int mm2s_taildesc;
   unsigned int mm2s_taildesc_msb;
   unsigned int mm2s_sg_ctl;
   unsigned int s2mm_dmacr;
   unsigned int s2mm_dmasr;
   unsigned int s2mm_curdesc;
   unsigned int s2mm_curdesc_msb;
   unsigned int s2mm_taildesc;
   unsigned int s2mm_taildesc_msb;
};

struct Buffer_Descriptor
{
   unsigned int nxtdesc;
   unsigned int nxtdesc_msb;
   unsigned int buffer_address;
   unsigned int buffer_address_msb;
   unsigned int reserved0;
   unsigned int reserved1;
   unsigned int control;
   unsigned int status;
   unsigned int curdesc;
   unsigned int curdesc_msb;
};

template <typename T>
class rSGDMA : public sc_module, public payload_event_queue_output_if<master_atom>
{
public:
   GenericMasterPort<32> m_axi_mm2s;
   GenericMasterPort<32> m_axi_sg;

   void notify(master_atom &);
   void notify_for_m_axi_mm2s(master_atom &);
   void notify_for_m_axi_sg(master_atom &);

   tlm_utils::simple_initiator_socket<rSGDMA> t_slave;
   tlm_utils::simple_target_socket<rSGDMA> uP2DMA;

   SC_HAS_PROCESS(rSGDMA);

private:
   typedef struct rSGDMA_config config;
   typedef struct SG_DMA_Register sg_register;
   typedef struct Buffer_Descriptor buff_desc;
   typedef tlm::tlm_generic_payload tlm_payload;

   typedef GenericMasterPort<32>::accessHandle transactionHandle;
   typedef GenericMasterPort<32>::phase phase;

   bool done;
   bool write_busy;
   std::list<transactionHandle> waiting_write; // deprecated

   sdl_fifo<buff_desc> *bd_fifo;
   sc_fifo<rSGDMA_config> *cfg_fifo;
   sdl_fifo<gs_uint32> *data_fifo;
   sdl_fifo<gs_uint32> *data_fifo_out;
   sdl_fifo<sc_time> *data_fifo_time;

   buff_desc bd; // Use at sg_port r channnel
   unsigned int bd_index;
   sg_register sg_reg;
   config cfg;
   unsigned int remain_len;
   unsigned int axi_addr;

   sc_event register_setting_complete;
   sc_event ev_sg_arrdy;
   sc_event ev_sg_rlast;
   sc_event ev_mm_transfer_done_for_1_bd;
   sc_event ev_sg_awrdy;
   sc_event ev_sg_b_handshake;
   sc_event bd_fifo_not_full;

   sc_event ev_mm_end_resp, ev_mm_end_req;
   sc_event wait_TREADY;
   sc_event ev_fifo_burst_free;

   int dma_max_busrt_num;
   int dma_max_busrt_length;

   transactionHandle *axi_pl;
   bool **flag_active_axi_pl;
   bool **flag_alloc_axi_pl;

   phase *axi_ph;

   T **mmdata;

   int dbg_num_trans_onair;

   GSDataType data;

   int bl;
   int bl_rsv;
   int mm2s_num_recieve_beat;
   int sg_tidx;
   int transfer_done_stack;
   int sg_rsv;
   int *cfg_size_stack;
   int *cfg_addr_stack;
   int stack_idx;
   int stack_idx2;
   int *match_sg_tidx_mm2s_addr;

   void sg_r_thread();
   void sg_w_thread();
   void axi_r_thread();
   void stream_thread();
   void pipeline_thread();

   bool take_trans(int &rt_tidx, int protocol_index, // pidx 0: mm2s, 1: sg r, 2: sg w
                   transactionHandle *payload, GenericMasterPort<32> *socket,
                   int nmot);

public:
   // constructor
   rSGDMA(const sc_module_name &name, int _dma_max_busrt_length,
          int _dma_max_busrt_num) : sc_module(name),
                                    m_axi_mm2s("iport"),
                                    m_axi_sg("sg_port"),
                                    t_slave("init_socket_binded_SIF"),
                                    uP2DMA("uP2DMA"),
                                    remain_len(0),
                                    axi_addr(0),
                                    dma_max_busrt_num(_dma_max_busrt_num),
                                    dma_max_busrt_length(_dma_max_busrt_length)
   {
      m_axi_mm2s.peq.out_port(*this);
      GSGPSocketConfig m_axi_mm2s_cnf;
      m_axi_mm2s_cnf.use_wr_resp = true;
      m_axi_mm2s.set_config(m_axi_mm2s_cnf);

      m_axi_sg.peq.out_port(*this);
      GSGPSocketConfig m_axi_sg_cnf;
      m_axi_sg_cnf.use_wr_resp = true;
      m_axi_sg.set_config(m_axi_sg_cnf);

      cfg_fifo = new sc_fifo<rSGDMA_config>("cfg_fifo", CFG_FIFO_SIZE);
      bd_fifo = new sdl_fifo<buff_desc>(SG_R_NMOT);

      data_fifo = new sdl_fifo<gs_uint32>(dma_max_busrt_length * (dma_max_busrt_num + 1));
      data_fifo_out = new sdl_fifo<gs_uint32>(dma_max_busrt_length * (dma_max_busrt_num + 1));
      data_fifo_time = new sdl_fifo<sc_time>(dma_max_busrt_length * (dma_max_busrt_num + 1));

      memset(&cfg, 0, sizeof(cfg));
      memset(&sg_reg, 0, sizeof(sg_reg));
      memset(&bd, 0, sizeof(buff_desc));

      t_slave.register_nb_transport_bw(this, &rSGDMA<T>::nb_transport_bw_from_SIF);
      uP2DMA.register_b_transport(this, &rSGDMA<T>::b_transport_fw);
      // const int tdatasize = 96;
      // const int datasize = 96;

      sg_reg.mm2s_dmasr = false & 0x1;
      sg_reg.s2mm_dmasr = false & 0x1;
      done = true;
      transfer_done_stack = 0;
      sg_rsv = 0;

      axi_pl = new transactionHandle[dma_max_busrt_num];

      flag_active_axi_pl = new bool *[3]; // SG_R, SG_W, MM2S or S2MM
      flag_alloc_axi_pl = new bool *[3];

#define MAX_NMOT MAX(dma_max_busrt_num, MAX(SG_R_NMOT, SG_W_NMOT))

      for (int pidx = 0; pidx < 3; pidx++) // protocol index: 0=mm2s, 1=sg r, 2= sg w
      {
         flag_active_axi_pl[pidx] = new bool[MAX_NMOT];
         flag_alloc_axi_pl[pidx] = new bool[MAX_NMOT];

         for (int tidx = 0; tidx < MAX_NMOT; tidx++)
         {
            flag_alloc_axi_pl[pidx][tidx] = false;
            flag_active_axi_pl[pidx][tidx] = false;
         }
      }

      axi_ph = new phase[dma_max_busrt_num];

      mmdata = new T *[MAX_NMOT];
      for (int tidx = 0; tidx < MAX_NMOT; tidx++)
      {
         mmdata[tidx] = new T[dma_max_busrt_length];

         for (int i = 0; i < dma_max_busrt_length; ++i)
         {
            mmdata[tidx][i] = 'z';
         }
      }

      dbg_num_trans_onair = 0;
      sg_tidx = 0;
      bd_index = 0;
      mm2s_num_recieve_beat = 0;
      match_sg_tidx_mm2s_addr = new int[SG_R_NMOT];
      cfg_size_stack = new int[2 * MAX_NMOT];
      cfg_addr_stack = new int[2 * MAX_NMOT];
      stack_idx = 0;
      stack_idx2 = 0;

      SC_THREAD(sg_r_thread);
      SC_THREAD(sg_w_thread);
      SC_THREAD(axi_r_thread);
      SC_THREAD(stream_thread);
      SC_THREAD(pipeline_thread);
   }

   // destructor
   ~rSGDMA()
   {
      delete cfg_fifo;
      delete bd_fifo;
      delete data_fifo;
      delete data_fifo_out;
      delete data_fifo_time;
      delete[] axi_pl;
      delete[] axi_ph;
      delete[] match_sg_tidx_mm2s_addr;
      delete[] cfg_size_stack;
      delete[] cfg_addr_stack;

      for (int sidx = 0; sidx < 3; sidx++)
      {
         delete[] flag_alloc_axi_pl[sidx];
         delete[] flag_active_axi_pl[sidx];
      }
      for (int tidx = 0; tidx < MAX_NMOT; tidx++)
         delete[] mmdata[tidx];

      delete[] mmdata;
      delete[] flag_alloc_axi_pl;
      delete[] flag_active_axi_pl;
   }

   inline void b_transport_fw(tlm::tlm_generic_payload &trans, sc_time &delay)
   {
      if (trans.get_command() == tlm::TLM_READ_COMMAND)
      {
         // wait(done);
         *(reinterpret_cast<bool *>(trans.get_data_ptr())) = done;
         if (done == true)
            DMA_LOG(1, "*** done ***");
      }
      else
      {
         wait(CYCLES(DMA_SET_GO_LATENCY));
         //         rSGDMA_config cfg_from_controller = *(reinterpret_cast<rSGDMA_config*>(trans.get_data_ptr()));
         //         cfg_fifo->write(cfg_from_controller);
         sg_reg = *(reinterpret_cast<sg_register *>(trans.get_data_ptr()));

         register_setting_complete.notify();
         DMA_LOG(1, "notify");
      }
   }

   inline tlm::tlm_sync_enum nb_transport_bw_from_bus(tlm::tlm_generic_payload &trans,
                                                      tlm::tlm_phase &phase, sc_time &delay)
   {

      return tlm::TLM_ACCEPTED;
   }

   inline tlm::tlm_sync_enum nb_transport_bw_from_SIF(tlm::tlm_generic_payload &trans,
                                                      tlm::tlm_phase &phase, sc_time &delay)
   {

      if (phase == tlm::BEGIN_RESP)
      {
         HANDSHAKE_PROTOCOL(1, 1, "TREADY");
         DMA_LOG(1, "TREADY");

         wait_TREADY.notify();
         return tlm::TLM_COMPLETED;
      }
      else
      {
         DMA_LOG(1, "*** error ***");
         SC_REPORT_FATAL("TLM-2", "Illegal anything phase received by tslave");
         return tlm::TLM_COMPLETED;
      }
   }
};

template <typename T>
void rSGDMA<T>::notify(master_atom &tc)
{
   if (_getMasterAccessHandle(tc).getMID() <= 2)
      notify_for_m_axi_mm2s(tc);
   else if (_getMasterAccessHandle(tc).getMID() <= 6)
      notify_for_m_axi_sg(tc);
   else
      cout << basename() << "Error: Please, Check port for SG and MM2S" << endl;
}

template <typename T>
void rSGDMA<T>::sg_r_thread()
{
   int tidx;

   transactionHandle *sg_r_payload = new transactionHandle[SG_R_NMOT];
   phase *sg_r_phase = new phase[SG_R_NMOT];

   while (1)
   {
      wait(register_setting_complete);

      if ((sg_reg.mm2s_dmacr & 0x1) == 0 ||
          sg_reg.mm2s_curdesc == 0 ||
          sg_reg.mm2s_taildesc == 0)
         continue;

      while (1)
      {
         if (bd_fifo->num_free() - sg_rsv < 1)
         {
            DMA_LOG(1, "bd_fifo_not_full_wait: " << bd_fifo->num_free() - sg_rsv - 1);
            wait(bd_fifo_not_full);
            DMA_LOG(1, "bd_fifo_not_full_wait_done: " << bd_fifo->num_free() - sg_rsv - 1);
         }

         while (!take_trans(tidx, 1, sg_r_payload, &m_axi_sg, SG_R_NMOT))
         {
            DMA_LOG(1, "SG R:no free trans");

            wait(ev_mm_transfer_done_for_1_bd);
            wait(CYCLES(TRANSFER_DONE_TO_NEXT_SG_ADDR_REQ_LATENCY));
         }

         sg_r_payload[tidx].setMAddr(sg_reg.mm2s_curdesc);
         sg_r_payload[tidx].setMBurstLength(8);
         sg_r_payload[tidx].setCnt(0);
         sg_r_payload[tidx].setLast(0);
         sg_r_payload[tidx].setMCmd(Generic_MCMD_RD);
         m_axi_sg.Request(sg_r_payload[tidx], sg_r_phase[tidx]);
         DMA_LOG(1, "SG_ARVALID, cur:" << hex << sg_reg.mm2s_curdesc << "tail:" << hex << sg_reg.mm2s_taildesc);

         sg_rsv++;
         wait(ev_sg_arrdy);

#ifdef VCD
         if (sc_time(VCD_TIME_BOUND_L, SC_NS) <= sc_time_stamp() && sc_time_stamp() <= sc_time(VCD_TIME_BOUND_H, SC_NS))
         {
            fout_vcd << "#" << fixed << (int)((sc_simulation_time() - VCD_TIME_BOUND_L) / PL_CLK_PRD) << endl;
            // fout_vcd<< "0$" <<endl;
            // fout_vcd<< "0%" <<endl;
            fout_vcd << "0$" << (sg_r_payload[tidx].getMID() == 4 ? "bb" : "aa") << endl;
            fout_vcd << "0%" << (sg_r_payload[tidx].getMID() == 4 ? "bb" : "aa") << endl;
         }
#endif
         if (sg_reg.mm2s_curdesc == sg_reg.mm2s_taildesc)
            break;

         wait(ev_sg_rlast);
      }
   }
}

template <typename T>
void rSGDMA<T>::sg_w_thread()
{
   int tidx;

   transactionHandle *sg_w_payload = new transactionHandle[SG_W_NMOT];
   phase *sg_w_phase = new phase[SG_W_NMOT];

   while (1)
   {
      DMA_LOG(1, "sg_w_thread_wait_transfer_done");
      if (transfer_done_stack == 0)
         wait(ev_mm_transfer_done_for_1_bd);
      wait(CYCLES(TRANSFER_DONE_TO_SG_AW_REQ_LATENCY));
      transfer_done_stack--;

      while (!take_trans(tidx, 2, sg_w_payload, &m_axi_sg, SG_W_NMOT))
      {
         DMA_LOG(1, "SG W:no free trans");

         wait(ev_sg_b_handshake);
      }

      // buff_desc    bd_temp = bd_fifo->read();
      buff_desc bd_temp;
      bd_fifo->nb_soft_read(bd_temp);

      sg_w_payload[tidx].setMAddr(bd_temp.curdesc + 0x1C);
      sg_w_payload[tidx].setMBurstLength(1);
      sg_w_payload[tidx].setCnt(0);
      sg_w_payload[tidx].setLast(0);
      sg_w_payload[tidx].setMCmd(Generic_MCMD_WR);
      sg_w_payload[tidx].setData32(bd_temp.control | 0x80000000);
      m_axi_sg.Request(sg_w_payload[tidx], sg_w_phase[tidx]);

      wait(ev_sg_awrdy);

#ifdef VCD
      if (sc_time(VCD_TIME_BOUND_L, SC_NS) <= sc_time_stamp() && sc_time_stamp() <= sc_time(VCD_TIME_BOUND_H, SC_NS))
      {
         fout_vcd << "#" << fixed << (int)((sc_simulation_time() - VCD_TIME_BOUND_L) / PL_CLK_PRD) << endl;
         // fout_vcd<< "0$" <<endl;
         // fout_vcd<< "0%" <<endl;
         fout_vcd << "0$$" << (sg_w_payload[tidx].getMID() == 4 ? "bb" : "aa") << endl;
         fout_vcd << "0%%" << (sg_w_payload[tidx].getMID() == 4 ? "bb" : "aa") << endl;
      }
#endif

      if (bd_temp.curdesc == sg_reg.mm2s_taildesc)
      {
         wait(ev_sg_b_handshake);
         sg_reg.mm2s_dmasr = false & 0x1;
      }
   }
}

template <typename T>
void rSGDMA<T>::axi_r_thread()
{
   // SC_SHOW_TIME("r_thread");
   bl_rsv = 0;

   while (1)
   {
      cfg = cfg_fifo->read();
      wait(CYCLES(SG_RLAST_TO_ADDR_REQ_LATENCY + 1));
      done = false;
      bl = MIN(dma_max_busrt_length, cfg.size);

      remain_len += cfg.size / sizeof(T);

      axi_addr = cfg.src_addr;

      // payload basic setting here
      cfg_size_stack[stack_idx] = cfg.size;
      cfg_addr_stack[stack_idx] = cfg.src_addr;
      stack_idx = (stack_idx + 1) % (2 * MAX_NMOT);

      for (int nt = bl; nt <= cfg.size; nt += bl)
      { // i think it is necessary
         DMA_LOG(1, "Sending request, burst len: " << bl);
         int tidx;
         while (!take_trans(tidx, 0, axi_pl, &m_axi_mm2s, dma_max_busrt_num))
         {
            DMA_LOG(1, "no free trans");

            wait(ev_mm_end_resp);
         }
         DMA_LOG(1, "data_fifo is almost full: " << data_fifo->num_free() - bl_rsv - 1);
         if (data_fifo->num_free() - bl_rsv - 1 < bl)
         { // to read, data_fifo have to enough space

            DMA_LOG(1, "burst free wait, " << data_fifo->num_free() - bl_rsv - 1);
            wait(ev_fifo_burst_free);
         }

         wait(CYCLES(MULTIPLE_OUT_LATENCY + 1));
         DMA_LOG(1, "ARVALID " << "tidx = " << dec << tidx);
         DMA_LOG(1, "0 " << data_fifo->num_free() - bl_rsv << ":" << bl << "/" << dma_max_busrt_num);

         axi_pl[tidx].setMAddr(axi_addr + (nt - bl) * 4);
         axi_pl[tidx].setMBurstLength(bl);
         axi_pl[tidx].setCnt(0);
         axi_pl[tidx].setLast(0);
         m_axi_mm2s.Request(axi_pl[tidx], axi_ph[tidx]);
         DMA_LOG(1, "mm2s ARVALID addr: 0x" << hex << (axi_addr + (nt - bl) * 4));
         // axi_addr += bl*4;

         bl_rsv += bl;
         // wait(ev_mm_end_resp); //notify function do wake this wait
         wait(ev_mm_end_req);

#ifdef VCD
         if (sc_time(VCD_TIME_BOUND_L, SC_NS) <= sc_time_stamp() && sc_time_stamp() <= sc_time(VCD_TIME_BOUND_H, SC_NS))
         {
            fout_vcd << "#" << fixed << (int)((sc_simulation_time() - VCD_TIME_BOUND_L) / PL_CLK_PRD) << endl;
            // fout_vcd<< "0$" <<endl;
            // fout_vcd<< "0%" <<endl;
            fout_vcd << "0$" << (axi_pl[tidx].getMID() ? "b" : "a") << endl;
            fout_vcd << "0%" << (axi_pl[tidx].getMID() ? "b" : "a") << endl;
         }
#endif

         DMA_LOG(1, "ARVALID, trans_onair: " << dbg_num_trans_onair << ", remain_len : " << cfg.size - nt);

         bl = MIN(dma_max_busrt_length, cfg.size - nt); // remain is smaller than blen
         // wait(CYCLES(MULTIPLE_OUT_LATENCY));
      }
      // wait(ev_mm_transfer_done_for_1_bd);
   }
}

template <typename T>
void rSGDMA<T>::stream_thread()
{
   // SC_SHOW_TIME("stream_thread start");
   unsigned int ti = 4;
   sc_time zt = SC_ZERO_TIME;
   tlm_payload t_pl;
   tlm::tlm_phase nph;
   while (1)
   {
      DMA_LOG(1, "waiting data_fifo: " << data_fifo->num_data());

      int one_fifo_data = data_fifo_out->read();
      DMA_LOG(1, "data" << one_fifo_data);
      DMA_LOG(1, "read data_fifo");

      DMA_LOG(1, "data_fifo is almost full: " << data_fifo->num_free() - bl_rsv - 1);
      if (data_fifo->num_free() - bl_rsv - 1 >= bl)
      {
         ev_fifo_burst_free.notify();
         DMA_LOG(1, "burst free notify, " << data_fifo->num_free() - bl_rsv - 1);
      }

      t_pl.set_command(tlm::TLM_WRITE_COMMAND);
      t_pl.set_data_length(ti /** sizeof(T)*/);
      t_pl.set_streaming_width(ti /** sizeof(T)*/); // debugging
      t_pl.set_address(0);
      t_pl.set_data_ptr(reinterpret_cast<unsigned char *>(&one_fifo_data));
      DMA_LOG(1, "TDATA = " << one_fifo_data);

      nph = tlm::BEGIN_REQ;

      DMA_LOG(1, "TVALID, remain_len : " << remain_len);
      HANDSHAKE_PROTOCOL(1, 1, "TVALID");

      tlm::tlm_sync_enum status = t_slave->nb_transport_fw(t_pl, nph, zt);

      DMA_LOG(1, "waiting TREADY");

      wait(wait_TREADY);

      if (--remain_len == 0)
      {
         done = true;
      }
   }
}

template <typename T>
void rSGDMA<T>::pipeline_thread()
{
   sc_time latency = SC_ZERO_TIME;
   while (1)
   {
      data_fifo->wait_not_empty();

      latency = data_fifo_time->read() + sc_time(CYCLES(RDMA_SIF_DATA_PIPLINE));

      while (latency > sc_time_stamp())
      {
         wait(CYCLES(1));
      }
      data_fifo_out->write(data_fifo->read());
   }
}

template <typename T>
bool rSGDMA<T>::take_trans(int &rt_tidx, int protocol_index,
                           transactionHandle *payload, GenericMasterPort<32> *socket,
                           int nmot)
{
   int tidx;
   for (tidx = 0; tidx < nmot; tidx++)
      if (!flag_active_axi_pl[protocol_index][tidx])
      {
         if (flag_alloc_axi_pl[protocol_index][tidx] == 0)
         {
            payload[tidx] = socket->create_transaction();
            payload[tidx].setMCmd(Generic_MCMD_RD);
            payload[tidx].setMData(gs::GSDataType::dtype(&((reinterpret_cast<unsigned char *>(mmdata[tidx]))[0]), 1));
            if (protocol_index == 2)
               payload[tidx].setTransID(tidx + SG_R_NMOT); // sg_r_mot == 3
            else
               payload[tidx].setTransID(tidx);
            flag_alloc_axi_pl[protocol_index][tidx] = 1;
         }
         flag_active_axi_pl[protocol_index][tidx] = 1;
         rt_tidx = tidx;
         dbg_num_trans_onair++;
         return 1;
      }
   return 0;
}

template <typename T>
void rSGDMA<T>::notify_for_m_axi_mm2s(master_atom &tc)
{

   transactionHandle tah = _getMasterAccessHandle(tc);
   phase p = _getPhase(tc);
   int tid = tah.getTransID();

   switch (p.state)
   {
   case GenericPhase::RequestAccepted:
      // We will handle no more than two transactions at a time
      HANDSHAKE_PROTOCOL(1, 2, "[BUS->DMA]ARREADY");
      DMA_LOG(1, "Receive ARREADY,  Wait RVALID for " << tah->getCnt() << "'s beat");

      ev_mm_end_req.notify();
      break;
   case GenericPhase::ResponseValid: // Should never receive this phase from initiator
      if (tah->getCnt() > tah->getMBurstLength())
      {

         SC_SHOW_TIME("Error " << tah->getCnt() << ": " << tah->getMBurstLength());

         SC_REPORT_ERROR(sc_core::SC_ID_INTERNAL_ERROR_, "Fell of the end of the burst?");
      }
      else
      {
         HANDSHAKE_PROTOCOL(1, 2, "[BUS->DMA]RVALID: mid" << tah->getMID() << "tid" << tah->getTransID());

#ifdef VCD
         if (sc_time(VCD_TIME_BOUND_L, SC_NS) <= sc_time_stamp() && sc_time_stamp() <= sc_time(VCD_TIME_BOUND_H, SC_NS))
         {
            fout_vcd << "#" << fixed << (int)((sc_simulation_time() - VCD_TIME_BOUND_L) / PL_CLK_PRD) << endl;
            // fout_vcd<< "1&" << endl;
            // fout_vcd<< "b" <<bitset<32>(tah->getData32()) << " #" <<endl;
            // fout_vcd<< tah->getLast() <<"(" << endl;
            fout_vcd << "1&" << (tah->getMID() ? "b" : "a") << endl;
            fout_vcd << "b" << bitset<32>(tah->getData32()) << " #" << (tah->getMID() ? "b" : "a") << endl;
            fout_vcd << tah->getLast() << "(" << (tah->getMID() ? "b" : "a") << endl;
         }
#endif

         DMA_LOG(1, "data_fifo push : " << tah->getData32() << " " << data_fifo->num_data() + 1);
         // data_fifo->write(&mmdata[tid][p.getBytesValid()-1]);

         if (data_fifo->nb_write(tah->getData32()) == 0)
            SC_REPORT_ERROR(sc_core::SC_ID_INTERNAL_ERROR_, "rSGDMA::notify(f/s), data fifo full");

         if (data_fifo_time->nb_write(sc_time_stamp()) == 0)
            SC_REPORT_ERROR(sc_core::SC_ID_INTERNAL_ERROR_, "rSGDMA::notify(f/s), data fifo full");

         bl_rsv--;
         mm2s_num_recieve_beat++;

         // if (p.getBytesValid()  < tah->getMBurstLength()) {
         if (!(tah->getLast()))
         {
            DMA_LOG(1, "Receive RVALID, Send RREADY for " << tah->getCnt() << "'s beat");

            m_axi_mm2s.AckResponse(tah, p, CYCLES(1));

#ifdef VCD
            if (sc_time(VCD_TIME_BOUND_L, SC_NS) <= sc_time_stamp() && sc_time_stamp() <= sc_time(VCD_TIME_BOUND_H, SC_NS))
            {
               fout_vcd << "#" << fixed << (int)((sc_simulation_time() - VCD_TIME_BOUND_L) / PL_CLK_PRD) << endl;
               // fout_vcd<< "1'" << endl;
               fout_vcd << "1'" << (tah->getMID() ? "b" : "a") << endl;
            }
#endif
         }
         else
         {
            DMA_LOG(1, "Got rlast");

            m_axi_mm2s.AckResponse(tah, p, CYCLES(1));

#ifdef VCD
            if (sc_time(VCD_TIME_BOUND_L, SC_NS) <= sc_time_stamp() && sc_time_stamp() <= sc_time(VCD_TIME_BOUND_H, SC_NS))
            {
               fout_vcd << "#" << fixed << (int)((sc_simulation_time() - VCD_TIME_BOUND_L) / PL_CLK_PRD) << endl;
               // fout_vcd<< "1'" << endl;
               fout_vcd << "1'" << (tah->getMID() ? "b" : "a") << endl;
            }
#endif

            flag_active_axi_pl[0][tid] = 0;
            dbg_num_trans_onair--;

            ev_mm_end_resp.notify(CYCLES(1));
            if (mm2s_num_recieve_beat == cfg_size_stack[stack_idx2])
            {
               transfer_done_stack++;
               // SC_SHOW_TIME("sgtidx done");
               for (sg_tidx = 0; sg_tidx < SG_R_NMOT; sg_tidx++)
               {
                  if (match_sg_tidx_mm2s_addr[sg_tidx] == cfg_addr_stack[stack_idx2])
                  {
                     // SC_SHOW_TIME("match: "<<match_sg_tidx_mm2s_addr[sg_tidx]
                     //    << "cfg: "<<cfg_addr_stack[stack_idx2] <<"sgtidx"<<sg_tidx);
                     flag_active_axi_pl[1][sg_tidx] = 0;
                     // sg_tidx = (sg_tidx+1)%(SG_R_NMOT);
                     break;
                  }
               }
               DMA_LOG(1, "trans pkt finish");
               mm2s_num_recieve_beat = 0;
               stack_idx2 = (stack_idx2 + 1) % (2 * MAX_NMOT);
               ev_mm_transfer_done_for_1_bd.notify();
            }
         }
      }

      if (tah->getMCmd() == Generic_MCMD_WR)
      {
         // SC_REPORT_ERROR( sc_core::SC_ID_INTERNAL_ERROR_, "Command not recognized" );
         assert(0);
      }
      else
      {
         // SC_REPORT_ERROR( sc_core::SC_ID_INTERNAL_ERROR_, "Command not recognized" );
      }
      break;
   default:
      cout << "Error - Memory peq_cb default case" << endl;
      SC_SHOW_TIME("notified phase = " << p.to_string());
      break;
   }
}

template <typename T>
void rSGDMA<T>::notify_for_m_axi_sg(master_atom &tc)
{

   transactionHandle tah = _getMasterAccessHandle(tc);
   phase p = _getPhase(tc);
   int tid = tah.getTransID();

   switch (p.state)
   {
   case GenericPhase::RequestAccepted:

      if (tah->getMCmd() == Generic_MCMD_RD)
      {
         // We will handle no more than two transactions at a time
         HANDSHAKE_PROTOCOL(1, 6, "[BUS->DMA]SG_ARREADY");
         DMA_LOG(1, "Receive ARREADY,  Wait RVALID for " << tah->getCnt() << "'s beat");

         ev_sg_arrdy.notify();
      }
      else
      {
         HANDSHAKE_PROTOCOL(2, 2, "[BUS->DMA]SG_AWREADY: mid" << tah->getMID() << "tid" << tah->getTransID() << "bl" << tah->getMBurstLength());
         DMA_LOG(1, "addr: " << tah->getMAddr());
         tah->setCnt(1);
         if (tah->getMBurstLength() == 1)
            tah->setLast(1);

         ev_sg_awrdy.notify();

         if (!write_busy)
         {
            m_axi_sg.SendData(tah, p, CYCLES(SG_AW_HANDSHAKE_TO_W_REQ_LATENCY));
            write_busy = true;
         }
         else
            waiting_write.push_back(tah);
      }
      break;
   case GenericPhase::ResponseValid: // Should never receive this phase from initiator
      if (tah->getMCmd() == Generic_MCMD_RD)
      {
         if (tah->getCnt() > tah->getMBurstLength())
         {
            SC_SHOW_TIME("Error " << tah->getCnt() << ": " << tah->getMBurstLength());
            SC_REPORT_ERROR(sc_core::SC_ID_INTERNAL_ERROR_, "Fell of the end of the burst?");
         }
         else
         {
            HANDSHAKE_PROTOCOL(1, 6, "[BUS->DMA]SG_RVALID");

#ifdef VCD
            if (sc_time(VCD_TIME_BOUND_L, SC_NS) <= sc_time_stamp() && sc_time_stamp() <= sc_time(VCD_TIME_BOUND_H, SC_NS))
            {
               fout_vcd << "#" << fixed << (int)((sc_simulation_time() - VCD_TIME_BOUND_L) / PL_CLK_PRD) << endl;
               // fout_vcd<< "1&" << endl;
               // fout_vcd<< "b" <<bitset<32>(tah->getData32()) << " #" <<endl;
               // fout_vcd<< tah->getLast() <<"(" << endl;
               fout_vcd << "1&" << (tah->getMID() == 4 ? "bb" : "aa") << endl;
               fout_vcd << "bb" << bitset<32>(tah->getData32()) << " #" << (tah->getMID() == 4 ? "bb" : "aa") << endl;
               fout_vcd << tah->getLast() << "(" << (tah->getMID() == 4 ? "bb" : "aa") << endl;
            }
#endif

            if (!(tah->getLast()))
            {
               DMA_LOG(1, "Receive SG_RVALID, Send RREADY for " << tah->getCnt() << " / " << tah->getMBurstLength() << "'s beat");

               ((unsigned int *)&bd)[bd_index++] = tah->getData32();

               m_axi_sg.AckResponse(tah, p, CYCLES(1));

#ifdef VCD
               if (sc_time(VCD_TIME_BOUND_L, SC_NS) <= sc_time_stamp() && sc_time_stamp() <= sc_time(VCD_TIME_BOUND_H, SC_NS))
               {
                  fout_vcd << "#" << fixed << (int)((sc_simulation_time() - VCD_TIME_BOUND_L) / PL_CLK_PRD) << endl;
                  // fout_vcd<< "1'" << endl;
                  fout_vcd << "1'" << (tah->getMID() == 4 ? "bb" : "aa") << endl;
               }
#endif
            }
            else
            {
               DMA_LOG(1, "Got SGrlast: bd:" << bd_fifo->num_data() << ", " << bd_fifo->num_free());

               ((unsigned int *)&bd)[bd_index] = tah->getData32();

               bd_index = 0;
               bd.curdesc = sg_reg.mm2s_curdesc;
               bd.curdesc_msb = sg_reg.mm2s_curdesc_msb;
               sg_reg.mm2s_curdesc = bd.nxtdesc;
               sg_reg.mm2s_curdesc_msb = bd.nxtdesc_msb;

               bd_fifo->write(bd);

               config cfg_temp;
               cfg_temp.size = (bd.control & 0x7FFFFF) / 4;
               cfg_temp.src_addr = bd.buffer_address;
               cfg_temp.dst_addr = 0;
               match_sg_tidx_mm2s_addr[tid] = cfg_temp.src_addr;
               cfg_fifo->write(cfg_temp);

               sg_rsv--;
               m_axi_sg.AckResponse(tah, p, CYCLES(1));
               ev_sg_rlast.notify(CYCLES(SG_RLAST_TO_NEXT_SG_ADDR_REQ_LATENCY));

#ifdef VCD
               if (sc_time(VCD_TIME_BOUND_L, SC_NS) <= sc_time_stamp() && sc_time_stamp() <= sc_time(VCD_TIME_BOUND_H, SC_NS))
               {
                  fout_vcd << "#" << fixed << (int)((sc_simulation_time() - VCD_TIME_BOUND_L) / PL_CLK_PRD) << endl;
                  // fout_vcd<< "1'" << endl;
                  fout_vcd << "1'" << (tah->getMID() == 4 ? "bb" : "aa") << endl;
               }
#endif
            }
         }
      }
      else
      {
         DMA_LOG(1, "ev_mm_end_resp notify in END_RESP phase");
         HANDSHAKE_PROTOCOL(2, 2, "[BUS->DMA]SG_BVALID: mid" << tah->getMID() << "tid" << tah->getTransID() << "bl" << tah->getMBurstLength());

         m_axi_sg.AckResponse(tah, p, CYCLES(1));
         flag_active_axi_pl[2][tid - SG_R_NMOT] = 0;

         bd_fifo->just_pop();
         if (bd_fifo->num_free() - sg_rsv >= 1)
         {
            DMA_LOG(1, "bd_fifo_not_full_notify: " << bd_fifo->num_free() - sg_rsv - 1);
            bd_fifo_not_full.notify();
         }

         ev_sg_b_handshake.notify(CYCLES(1));

#ifdef VCD
         if (sc_time(VCD_TIME_BOUND_L, SC_NS) <= sc_time_stamp() && sc_time_stamp() <= sc_time(VCD_TIME_BOUND_H, SC_NS))
         {
            fout_vcd << "#" << fixed << (int)((sc_simulation_time() - VCD_TIME_BOUND_L) / PL_CLK_PRD) << endl;
            fout_vcd << "1~" << (tah->getMID() == 4 ? "bb" : "aa") << endl;
            fout_vcd << "1~~" << (tah->getMID() == 4 ? "bb" : "aa") << endl;
         }
#endif
      }
      break;

   case GenericPhase::DataAccepted: // Should never receive this phase from initiator
      HANDSHAKE_PROTOCOL(2, 2, "[BUS->DMA]SG_WREADY: mid" << tah->getMID() << "tid" << tah->getTransID() << "bl" << tah->getMBurstLength());
      DMA_LOG(1, "WREADY receive");

#ifdef VCD
      if (sc_time(VCD_TIME_BOUND_L, SC_NS) <= sc_time_stamp() && sc_time_stamp() <= sc_time(VCD_TIME_BOUND_H, SC_NS))
      {
         fout_vcd << "#" << fixed << (int)((sc_simulation_time() - VCD_TIME_BOUND_L) / PL_CLK_PRD) << endl;
         fout_vcd << "0''" << (tah->getMID() == 4 ? "bb" : "aa") << endl;
         fout_vcd << "0&&" << (tah->getMID() == 4 ? "bb" : "aa") << endl;
      }
#endif

      if (tah->getCnt() != tah->getMBurstLength())
      {
         SC_REPORT_ERROR(sc_core::SC_ID_INTERNAL_ERROR_, "Fell of the end of the burst?");
      }
      else
      {
         DMA_LOG(1, "Sent all data");
         write_busy = false;
         if (!waiting_write.empty())
         {
            m_axi_sg.SendData(waiting_write.front(), p, CYCLES(SG_AW_HANDSHAKE_TO_W_REQ_LATENCY));
            waiting_write.pop_front();
            write_busy = true;
         }

#ifdef VCD
         if (sc_time(VCD_TIME_BOUND_L, SC_NS) <= sc_time_stamp() && sc_time_stamp() <= sc_time(VCD_TIME_BOUND_H, SC_NS))
         {
            fout_vcd << "#" << fixed << (int)((sc_simulation_time() - VCD_TIME_BOUND_L) / PL_CLK_PRD) << endl;
            fout_vcd << "0((" << (tah->getMID() == 4 ? "bb" : "aa") << endl;
         }
#endif
      }
      break;

   default:
      SC_SHOW_TIME("notified phase = " << p.to_string());
      break;
   }
}
