// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

module mempool_cc
  import snitch_pkg::meta_id_t;
  import fpnew_pkg::*;
#(
  parameter logic [31:0] BootAddr   = 32'h0000_1000,
  parameter logic [31:0] MTVEC      = BootAddr,
  parameter bit          RVE        = 0,  // Reduced-register extension
  parameter bit          RVM        = 1,  // Enable IntegerMmultiplication & Division Extension
  parameter bit          FP_EN      = `ifdef FPU `FPU `else 0 `endif,
  parameter bit          RVF        = `ifdef FPU `FPU `else 0 `endif,
  parameter bit          RVD        = `ifdef FPU `FPU `else 0 `endif,
  parameter bit RegisterOffloadReq  = 1,
  parameter bit RegisterOffloadResp = 1,
  parameter bit RegisterTCDMReq     = 0,
  parameter bit RegisterTCDMResp    = 0,
  parameter int NumTCDMConn         = 2
) (
  input  logic               clk_i,
  input  logic               rst_i,
  input  logic [31:0]        hart_id_i,
  // Instruction Port
  output logic [31:0]        inst_addr_o,
  input  logic [31:0]        inst_data_i,
  output logic               inst_valid_o,
  input  logic               inst_ready_i,
  // TCDM Ports
  output logic     [NumTCDMConn-1:0][31:0] data_qaddr_o,
  output logic     [NumTCDMConn-1:0]       data_qwrite_o,
  output logic     [NumTCDMConn-1:0][3:0]  data_qamo_o,
  output logic     [NumTCDMConn-1:0][31:0] data_qdata_o,
  output logic     [NumTCDMConn-1:0][3:0]  data_qstrb_o,
  output meta_id_t [NumTCDMConn-1:0]       data_qid_o,
  output logic     [NumTCDMConn-1:0]       data_qvalid_o,
  input  logic     [NumTCDMConn-1:0]       data_qready_i,
  input  logic     [NumTCDMConn-1:0][31:0] data_pdata_i,
  input  logic     [NumTCDMConn-1:0]       data_perror_i,
  input  meta_id_t [NumTCDMConn-1:0]       data_pid_i,
  input  logic     [NumTCDMConn-1:0]       data_pvalid_i,
  output logic     [NumTCDMConn-1:0]       data_pready_o,
  input  logic                             wake_up_sync_i,
  // Core event strobes
  output snitch_pkg::core_events_t core_events_o
);

  // Data port signals
  snitch_pkg::dreq_t  data_req_fpu_d, data_req_fpu_q;
  snitch_pkg::dresp_t data_resp_fpu_d, data_resp_fpu_q;
  snitch_pkg::dreq_t  data_req_snitch_d, data_req_snitch_q;
  snitch_pkg::dresp_t data_resp_snitch_d, data_resp_snitch_q;

  snitch_pkg::dreq_t [NumTCDMConn-1:0]  data_req_d, data_req_q;
  snitch_pkg::dresp_t [NumTCDMConn-1:0] data_resp_d, data_resp_q;

  logic data_req_fpu_d_valid, data_req_fpu_d_ready, data_resp_fpu_d_valid, data_resp_fpu_d_ready;
  logic data_req_fpu_q_valid, data_req_fpu_q_ready, data_resp_fpu_q_valid, data_resp_fpu_q_ready;
  logic data_req_snitch_d_valid, data_req_snitch_d_ready, data_resp_snitch_d_valid, data_resp_snitch_d_ready;
  logic data_req_snitch_q_valid, data_req_snitch_q_ready, data_resp_snitch_q_valid, data_resp_snitch_q_ready;

  logic [NumTCDMConn-1:0] data_req_d_valid, data_req_d_ready, data_resp_d_valid, data_resp_d_ready;
  logic [NumTCDMConn-1:0] data_req_q_valid, data_req_q_ready, data_resp_q_valid, data_resp_q_ready;

  // Accelerator signals
  snitch_pkg::acc_req_t  acc_req_d,  acc_req_q;
  snitch_pkg::acc_resp_t acc_resp_d, acc_resp_q;
  snitch_pkg::acc_resp_t ipu_resp, fpu_seq;

  logic acc_req_d_valid, acc_req_d_ready, acc_resp_d_valid, acc_resp_d_ready;
  logic acc_req_q_valid, acc_req_q_ready, acc_resp_q_valid, acc_resp_q_ready;

  logic ipu_dvalid, ipu_dready, fpu_dvalid, fpu_dready;
  logic ipu_qvalid, ipu_qready, fpu_qvalid, fpu_qready;

  // Snitch Integer Core
  snitch #(
    .BootAddr ( BootAddr ),
    .MTVEC    ( MTVEC    ),
    .RVE      ( RVE            ),
    .RVM      ( RVM            ),
    .FP_EN    ( FP_EN          ),
    .RVF      ( RVF            ),
    .RVD      ( RVD            )
  ) i_snitch (
    .clk_i                                         ,
    .rst_i                                         ,
    .hart_id_i                                     ,
    .inst_addr_o                                   ,
    .inst_data_i                                   ,
    .inst_valid_o                                  ,
    .inst_ready_i                                  ,
    .acc_qaddr_o            ( acc_req_d.addr      ),
    .acc_qid_o              ( acc_req_d.id        ),
    .acc_qdata_op_o         ( acc_req_d.data_op   ),
    .acc_qdata_arga_o       ( acc_req_d.data_arga ),
    .acc_qdata_argb_o       ( acc_req_d.data_argb ),
    .acc_qdata_argc_o       ( acc_req_d.data_argc ),
    .acc_qvalid_o           ( acc_req_d_valid     ),
    .acc_qready_i           ( acc_req_d_ready     ),
    .acc_pdata_i            ( acc_resp_q.data     ),
    .acc_pid_i              ( acc_resp_q.id       ),
    .acc_perror_i           ( acc_resp_q.error    ),
    .acc_pvalid_i           ( acc_resp_q_valid    ),
    .acc_pready_o           ( acc_resp_q_ready    ),
    .data_qaddr_o           ( data_req_snitch_d.addr   ),
    .data_qwrite_o          ( data_req_snitch_d.write  ),
    .data_qamo_o            ( data_req_snitch_d.amo    ),
    .data_qdata_o           ( data_req_snitch_d.data   ),
    .data_qstrb_o           ( data_req_snitch_d.strb   ),
    .data_qid_o             ( data_req_snitch_d.id     ),
    .data_qvalid_o          ( data_req_snitch_d_valid  ),
    .data_qready_i          ( data_req_snitch_d_ready  ),
    .data_pdata_i           ( data_resp_snitch_q.data  ),
    .data_perror_i          ( data_resp_snitch_q.error ),
    .data_pid_i             ( data_resp_snitch_q.id    ),
    .data_pvalid_i          ( data_resp_snitch_q_valid ),
    .data_pready_o          ( data_resp_snitch_q_ready ),
    .wake_up_sync_i         ( wake_up_sync_i      ),
    .core_events_o          ( core_events_o       ),
    .fpu_rnd_mode_o         ( /* Unused */        ),
    .fpu_status_i           ( '0                  )
  );

  // Assign Snitch data interface
  assign data_req_d_valid[0] = data_req_snitch_d_valid;
  assign data_req_d_ready[0] = data_req_snitch_d_ready;
  assign data_req_d[0] = data_req_snitch_d;
  assign data_resp_snitch_q = data_resp_q[1];
  assign data_resp_snitch_q_valid = data_resp_q_valid[1];
  assign data_resp_snitch_q_ready = data_resp_q_ready[1];

  // Cut off-loading request path
  spill_register #(
    .T      ( snitch_pkg::acc_req_t ),
    .Bypass ( !RegisterOffloadReq   )
  ) i_spill_register_acc_req (
    .clk_i   ,
    .rst_ni  ( ~rst_i          ),
    .valid_i ( acc_req_d_valid ),
    .ready_o ( acc_req_d_ready ),
    .data_i  ( acc_req_d       ),
    .valid_o ( acc_req_q_valid ),
    .ready_i ( acc_req_q_ready ),
    .data_o  ( acc_req_q       )
  );

  // Cut off-loading response path
  spill_register #(
    .T      ( snitch_pkg::acc_resp_t ),
    .Bypass ( !RegisterOffloadResp   )
  ) i_spill_register_acc_resp (
    .clk_i                       ,
    .rst_ni  ( ~rst_i           ),
    .valid_i ( acc_resp_d_valid ),
    .ready_o ( acc_resp_d_ready ),
    .data_i  ( acc_resp_d       ),
    .valid_o ( acc_resp_q_valid ),
    .ready_i ( acc_resp_q_ready ),
    .data_o  ( acc_resp_q       )
  );

  // Accelerator Demux Port
  stream_demux #(
    .N_OUP ( 5 )
  ) i_stream_demux_offload (
    .inp_valid_i  ( acc_req_qvalid_q               ),
    .inp_ready_o  ( acc_req_qready_q               ),
    .oup_sel_i    ( acc_req_q.addr[$clog2(5)-1:0]  ),
    .oup_valid_o  ( {ipu_qvalid, fpu_qvalid}       ),
    .oup_ready_i  ( {ipu_qready, fpu_qready}       )
  );

  stream_arbiter #(
    .DATA_T      ( snitch_pkg::acc_resp_t ),
    .N_INP       ( 5          )
  ) i_stream_arbiter_offload (
    .clk_i       ( clk_i                            ),
    .rst_ni      ( rst_ni                           ),
    .inp_data_i  ( {ipu_resp,   fpu_seq      }      ),
    .inp_valid_i ( {ipu_dvalid, fpu_dvalid   }      ),
    .inp_ready_o ( {ipu_dready, fpu_dready   }      ),
    .oup_data_o  ( acc_resp_d        ),
    .oup_valid_o ( acc_resp_d_valid  ),
    .oup_ready_i ( acc_resp_d_ready  )
  );

  // Snitch IPU accelerator
  snitch_ipu #(
    .IdWidth ( 5 )
  ) i_snitch_ipu (
    .clk_i                                   ,
    .rst_i                                   ,
    .acc_qaddr_i      ( acc_req_q.addr      ),
    .acc_qid_i        ( acc_req_q.id        ),
    .acc_qdata_op_i   ( acc_req_q.data_op   ),
    .acc_qdata_arga_i ( acc_req_q.data_arga ),
    .acc_qdata_argb_i ( acc_req_q.data_argb ),
    .acc_qdata_argc_i ( acc_req_q.data_argc ),
    .acc_qvalid_i     ( ipu_qvalid          ),
    .acc_qready_o     ( ipu_qready          ),
    .acc_pdata_o      ( ipu_resp.data       ),
    .acc_pid_o        ( ipu_resp.id         ),
    .acc_perror_o     ( ipu_resp.error      ),
    .acc_pvalid_o     ( ipu_dvalid          ),
    .acc_pready_i     ( ipu_dready          )
  );

  // Snitch FPU sub-system
  snitch_fp_ss #(
    .FPUImplementation      ( '0                     ),
    .NumFPOutstandingLoads  (  0                     ),
    .NumFPOutstandingMem    (  0                     ),
    .NumFPUSequencerInstr   (  0                     ),
    .NumSsrs                (  0                     ),
    .SsrRegs                ( '0                     ),
    .RegisterSequencer      ( 0                      ),
    .Xfrep                  ( 1                      ),
    .Xssr                   ( 1                      ),
    .RVF                    ( RVF                    ),
    .RVD                    ( RVD                    )
  ) i_snitch_fp_ss (
    .clk_i,
    .rst_i            ( ~rst_ni  ),
    // pragma translate_off
    .trace_port_o            ( fpu_trace           ),
    .sequencer_tracer_port_o ( fpu_sequencer_trace ),
    // pragma translate_on
    .acc_qaddr_i             ( acc_req_q.addr        ),
    .acc_qid_i               ( acc_req_q.id          ),
    .acc_qdata_op_i          ( acc_req_q.data_op     ),
    .acc_qdata_arga_i        ( acc_req_q.data_arga   ),
    .acc_qdata_argb_i        ( acc_req_q.data_argb   ),
    .acc_qdata_argc_i        ( acc_req_q.data_argc   ),
    .acc_qvalid_i            ( fpu_qvalid            ),
    .acc_qready_o            ( fpu_qready            ),
    .acc_pdata_o             ( fpu_resp.data         ),
    .acc_pid_o               ( fpu_resp.id           ),
    .acc_perror_o            ( fpu_resp.error        ),
    .acc_pvalid_o            ( fpu_dvalid            ),
    .acc_pready_i            ( fpu_dready            ),
    .data_qaddr_o            ( data_req_fpu_d.addr   ),
    .data_qwrite_o           ( data_req_fpu_d.write  ),
    .data_qamo_o             ( data_req_fpu_d.amo    ),
    .data_qdata_o            ( data_req_fpu_d.data   ),
    .data_qstrb_o            ( data_req_fpu_d.strb   ),
    .data_qid_o              ( data_req_fpu_d.id     ),
    .data_qvalid_o           ( data_req_fpu_d_valid  ),
    .data_qready_i           ( data_req_fpu_d_ready  ),
    .data_pdata_i            ( data_resp_fpu_q.data  ),
    .data_perror_i           ( data_resp_fpu_q.error ),
    .data_pid_i              ( data_resp_fpu_q.id    ),
    .data_pvalid_i           ( data_resp_fpu_q_valid ),
    .data_pready_o           ( data_resp_fpu_q_ready ),
    .fpu_rnd_mode_i          ( fpu_rnd_mode          ),
    .fpu_fmt_mode_i          ( /* Unused */          ),
    .fpu_status_o            ( fpu_status            ),
    .ssr_raddr_o             ( /* Unused */          ),
    .ssr_rdata_i             ( /* Unused */          ),
    .ssr_rvalid_o            ( /* Unused */          ),
    .ssr_rready_i            ( /* Unused */          ),
    .ssr_rdone_o             ( /* Unused */          ),
    .ssr_waddr_o             ( /* Unused */          ),
    .ssr_wdata_o             ( /* Unused */          ),
    .ssr_wvalid_o            ( /* Unused */          ),
    .ssr_wready_i            ( /* Unused */          ),
    .ssr_wdone_o             ( /* Unused */          ),
    .streamctl_done_i        ( /* Unused */          ),
    .streamctl_valid_i       ( /* Unused */          ),
    .streamctl_ready_o       ( /* Unused */          ),
    .core_events_o
  );

  // Assign FPU data interface
  assign data_req_d[1] = data_req_fpu_d;
  assign data_req_d_valid[1] = data_req_fpu_d_valid;
  assign data_req_d_ready[1] = data_req_fpu_d_ready;
  assign data_resp_fpu_q = data_resp_q[1];
  assign data_resp_fpu_q_valid = data_resp_q_valid[1];
  assign data_resp_fpu_q_ready = data_resp_q_ready[1];

  // Cut TCDM data request path
  spill_register #(
    .T      ( snitch_pkg::dreq_t ),
    .Bypass ( !RegisterTCDMReq   )
  ) i_spill_register_tcdm_req (
    .clk_i                       ,
    .rst_ni  ( ~rst_i           ),
    .valid_i ( data_req_d_valid ),
    .ready_o ( data_req_d_ready ),
    .data_i  ( data_req_d       ),
    .valid_o ( data_req_q_valid ),
    .ready_i ( data_req_q_ready ),
    .data_o  ( data_req_q       )
  );

  // Cut TCDM data response path
  spill_register #(
    .T      ( snitch_pkg::dresp_t ),
    .Bypass ( !RegisterTCDMResp   )
  ) i_spill_register_tcdm_resp (
    .clk_i                        ,
    .rst_ni  ( ~rst_i            ),
    .valid_i ( data_resp_d_valid ),
    .ready_o ( data_resp_d_ready ),
    .data_i  ( data_resp_d       ),
    .valid_o ( data_resp_q_valid ),
    .ready_i ( data_resp_q_ready ),
    .data_o  ( data_resp_q       )
  );

  // Assign TCDM data interface
  genvar idx_TCDMport;
  for ( idx_TCDMport = 0; idx_TCDMport < NumTCDMConn; idx_TCDMport++) begin
    assign data_qaddr_o[idx_TCDMport]      = data_req_q[idx_TCDMport].addr;
    assign data_qwrite_o[idx_TCDMport]     = data_req_q[idx_TCDMport].write;
    assign data_qamo_o[idx_TCDMport]       = data_req_q[idx_TCDMport].amo;
    assign data_qdata_o[idx_TCDMport]      = data_req_q[idx_TCDMport].data;
    assign data_qstrb_o[idx_TCDMport]      = data_req_q[idx_TCDMport].strb;
    assign data_qid_o[idx_TCDMport]        = data_req_q[idx_TCDMport].id;
    assign data_qvalid_o[idx_TCDMport]     = data_req_q_valid[idx_TCDMport];
    assign data_req_q_ready[idx_TCDMport] = data_qready_i[idx_TCDMport];
    assign data_resp_d[idx_TCDMport].data  = data_pdata_i[idx_TCDMport];
    assign data_resp_d[idx_TCDMport].id    = data_pid_i[idx_TCDMport];
    assign data_resp_d[idx_TCDMport].write = 'x; // Don't care here
    assign data_resp_d[idx_TCDMport].error = data_perror_i[idx_TCDMport];
    assign data_resp_d_valid[idx_TCDMport] = data_pvalid_i[idx_TCDMport];
    assign data_pready_o[idx_TCDMport]     = data_resp_d_ready[idx_TCDMport];
  end

  // --------------------------
  // Tracer
  // --------------------------
  // pragma translate_off
  int f;
  string fn;
  logic [63:0] cycle;
  int unsigned stall, stall_ins, stall_raw, stall_lsu, stall_acc;

  always_ff @(posedge rst_i) begin
    if(rst_i) begin
      // Format in hex because vcs and vsim treat decimal differently
      // Format with 8 digits because Verilator does not support anything else
      $sformat(fn, "trace_hart_0x%08x.dasm", hart_id_i);
      f = $fopen(fn, "w");
      $display("[Tracer] Logging Hart %d to %s", hart_id_i, fn);
    end
  end

  typedef enum logic [1:0] {SrcSnitch =  0, SrcFpu = 1, SrcFpuSeq = 2} trace_src_e;
  localparam int SnitchTrace = `ifdef SNITCH_TRACE `SNITCH_TRACE `else 0 `endif;

  always_ff @(posedge clk_i or posedge rst_i) begin
      automatic string trace_entry;
      automatic string extras_str;

      if (!rst_i) begin
        cycle <= cycle + 1;
        // Trace snitch iff:
        // Tracing enabled by CSR register
        // we are not stalled <==> we have issued and processed an instruction (including offloads)
        // OR we are retiring (issuing a writeback from) a load or accelerator instruction
        if ((i_snitch.csr_trace_q || SnitchTrace) && (!i_snitch.stall || i_snitch.retire_load || i_snitch.retire_acc)) begin
          // Manual loop unrolling for Verilator
          // Data type keys for arrays are currently not supported in Verilator
          extras_str = "{";
          // State
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "source",      SrcSnitch);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "stall",       i_snitch.stall);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "stall_tot",   stall);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "stall_ins",   stall_ins);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "stall_raw",   stall_raw);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "stall_lsu",   stall_lsu);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "stall_acc",   stall_acc);
          // Decoding
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "rs1",         i_snitch.rs1);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "rs2",         i_snitch.rs2);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "rd",          i_snitch.rd);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "is_load",     i_snitch.is_load);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "is_store",    i_snitch.is_store);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "is_branch",   i_snitch.is_branch);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "pc_d",        i_snitch.pc_d);
          // Operands
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "opa",         i_snitch.opa);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "opb",         i_snitch.opb);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "opa_select",  i_snitch.opa_select);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "opb_select",  i_snitch.opb_select);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "opc_select",  i_snitch.opc_select);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "write_rd",    i_snitch.write_rd);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "csr_addr",    i_snitch.inst_data_i[31:20]);
          // Pipeline writeback
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "writeback",   i_snitch.alu_writeback);
          // Load/Store
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "gpr_rdata_1", i_snitch.gpr_rdata[1]);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "gpr_rdata_2", i_snitch.gpr_rdata[2]);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "ls_size",     i_snitch.ls_size);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "ld_result_32",i_snitch.ld_result[31:0]);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "lsu_rd",      i_snitch.lsu_rd);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "retire_load", i_snitch.retire_load);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "alu_result",  i_snitch.alu_result);
          // Atomics
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "ls_amo",      i_snitch.ls_amo);
          // Accumulator
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "retire_acc",  i_snitch.retire_acc);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "acc_pid",     i_snitch.acc_pid_i);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "acc_pdata_32",i_snitch.acc_pdata_i[31:0]);
          // FPU offload
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "fpu_offload", 1'b0);
          extras_str = $sformatf("%s'%s': 0x%8x, ", extras_str, "is_seq_insn", 1'b0);
          extras_str = $sformatf("%s}", extras_str);

          $sformat(trace_entry, "%t %8d 0x%h DASM(%h) #; %s\n",
              $time, cycle, i_snitch.pc_q, i_snitch.inst_data_i, extras_str);
          $fwrite(f, trace_entry);
        end

        // Reset all stalls when we execute an instruction
        if (!i_snitch.stall) begin
            stall <= 0;
            stall_ins <= 0;
            stall_raw <= 0;
            stall_lsu <= 0;
            stall_acc <= 0;
        end else begin
          // We are currently stalled, let's count the stall causes
          if (i_snitch.stall) begin
            stall <= stall + 1;
          end
          if ((!i_snitch.inst_ready_i) && (i_snitch.inst_valid_o)) begin
            stall_ins <= stall_ins + 1;
          end
          if ((!i_snitch.operands_ready) || (!i_snitch.dst_ready)) begin
            stall_raw <= stall_raw + 1;
          end
          if (i_snitch.lsu_stall) begin
            stall_lsu <= stall_lsu + 1;
          end
          if (i_snitch.acc_stall) begin
            stall_acc <= stall_acc + 1;
          end
        end
      end else begin
        cycle <= '0;
        stall <= 0;
        stall_ins <= 0;
        stall_raw <= 0;
        stall_lsu <= 0;
        stall_acc <= 0;
      end
    end

  final begin
    $fclose(f);
  end
  // pragma translate_on

endmodule
