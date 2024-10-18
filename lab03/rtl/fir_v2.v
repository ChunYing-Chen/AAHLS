module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    output  wire                     awready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,

    output  wire                     wready,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,

    output  wire                     arready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,

    input   wire                     rready,
    output  wire                     rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata,

    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  wire                     ss_tready, 
    input   wire                     sm_tready, 
    output  wire                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);
    localparam S_IDLE   = 3'b000;
    localparam S_START  = 3'b001;
    localparam S_CALCP  = 3'b010;
    localparam S_CALCN  = 3'b011;
    localparam S_SWAIT  = 3'b100;
    localparam S_MWAIT  = 3'b101;
    localparam S_FINISH = 3'b110;
    
    integer                 i;
    //--AP_CTRL & data_length & tap_write---------------------------------------------------
    reg                     ap_start;
    reg                     ap_done;
    reg                     ap_idle;
    reg  [pDATA_WIDTH-1:0]  data_length;

    reg                     ap_start_w;
    reg                     ap_done_w;
    reg                     ap_idle_w;
    reg  [pDATA_WIDTH-1:0]  data_length_w;

    reg                     data_length_setting;
    reg                     tap_setting             [Tape_Num-1:0];

    reg                     data_length_setting_w;
    reg                     tap_setting_w           [Tape_Num-1:0];

    reg                     tap_setting_check       [Tape_Num:0];
    //--STATE-------------------------------------------------------------------------------
    reg  [2:0]              state;
    reg  [2:0]              next_state;
    //--TAP_RAM-----------------------------------------------------------------------------
    reg  [3:0]              tap_WE_reg;
    reg                     tap_EN_reg;
    reg  [pDATA_WIDTH-1:0]  tap_Di_reg;
    reg  [pADDR_WIDTH-1:0]  tap_A_reg;

    reg  [3:0]              tap_WE_wire;
    reg                     tap_EN_wire;
    reg  [pDATA_WIDTH-1:0]  tap_Di_wire;
    reg  [pADDR_WIDTH-1:0]  tap_A_wire;
    //--DATA_RAM----------------------------------------------------------------------------
    reg  [3:0]              data_WE_reg;
    reg                     data_EN_reg;
    reg  [pDATA_WIDTH-1:0]  data_Di_reg;
    reg  [pADDR_WIDTH-1:0]  data_A_reg;

    reg  [3:0]              data_WE_wire;
    reg                     data_EN_wire;
    reg  [pDATA_WIDTH-1:0]  data_Di_wire;
    reg  [pADDR_WIDTH-1:0]  data_A_wire;
    //--AXI_LITE_READ-----------------------------------------------------------------------
    wire [pDATA_WIDTH-1:0]  axi_lite_data;
    wire [pADDR_WIDTH-1:0]  axi_lite_addr;
    //--AXIS--------------------------------------------------------------------------------
    reg                     ss_tready_reg;
    reg                     sm_tvalid_reg;
    reg [(pDATA_WIDTH-1):0] sm_tdata_reg;
    reg                     sm_tlast_reg;
    //--COUNTER-----------------------------------------------------------------------------
    reg  [pDATA_WIDTH-1:0]  data_counter;
    reg  [$clog2(Tape_Num)-1:0] counter;
    reg  [$clog2(Tape_Num)-1:0] data_iterator;
    reg  [$clog2(Tape_Num)-1:0] tap_iterator;
    //--X_IN--------------------------------------------------------------------------------
    reg  [pDATA_WIDTH-1:0]  x_in;
    reg                     x_in_check;
    reg                     x_last;
    //--Y_OUT-------------------------------------------------------------------------------
    reg                     y_out_check;
    //--ALU---------------------------------------------------------------------------------
    wire [pDATA_WIDTH-1:0]  mul_in_a;
    wire [pDATA_WIDTH-1:0]  mul_in_b;
    wire [pDATA_WIDTH-1:0]  mul_tmp;
    reg  [pDATA_WIDTH-1:0]  calc_tmp;
    wire  [pDATA_WIDTH-1:0] calc_tmp_w;
    //--READ_AP-----------------------------------------------------------------------------
    wire                    r_ap_en;
    wire                    r_tap_valid;
    wire [pADDR_WIDTH-1:0]  r_tap_addr;

    
    //-------unused!---------------------
    wire                    mul_overflow;
    wire                    add_overflow;
    //-----------------------------------

/*----------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------*/

    //--TAP_RAM-----------------------------------------------------------------------------
    assign tap_WE = tap_WE_reg;
    assign tap_EN = tap_EN_reg;
    assign tap_Di = tap_Di_reg;
    assign tap_A  = tap_A_reg;
    //--DATA_RAM----------------------------------------------------------------------------
    assign data_WE = data_WE_reg;
    assign data_EN = data_EN_reg;
    assign data_Di = data_Di_reg;
    assign data_A  = data_A_reg;
    //--AXIS--------------------------------------------------------------------------------
    assign ss_tready = ss_tready_reg;
    assign sm_tvalid = sm_tvalid_reg;
    assign sm_tlast  = sm_tlast_reg;
    assign sm_tdata  = sm_tdata_reg;
    //--ALU---------------------------------------------------------------------------------
    assign mul_in_a = (state == S_CALCN)? x_in : data_Do;
    assign mul_in_b = tap_Do;
/*----------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------*/
    S_AXI_LITE #(
        .pADDR_WIDTH(pADDR_WIDTH),
        .pDATA_WIDTH(pDATA_WIDTH)
    )s_axi_lite(
        .awready(awready),
        .awvalid(awvalid),
        .awaddr(awaddr),

        .wready(wready),
        .wvalid(wvalid),
        .wdata(wdata),

        .arready(arready),
        .arvalid(arvalid),
        .araddr(araddr),

        .rready(rready),
        .rvalid(rvalid),
        .rdata(rdata),

        .axi_lite_data(axi_lite_data),
        .axi_lite_addr(axi_lite_addr),
        .r_ap_data({ap_idle, ap_done, ap_start}),
        .r_tap_valid(r_tap_valid),
        .r_tap_data((next_state == S_IDLE) ? tap_Do : {(pDATA_WIDTH){1'b1}}),
        .r_tap_addr(r_tap_addr),
        .r_ap_en(r_ap_en),
        .r_data_length(data_length),
        .axis_clk(axis_clk),
        .axis_rst_n(axis_rst_n)
    );

    MUL #(
        .pDATA_WIDTH(pDATA_WIDTH)
    )mul(
        
        .mul_in_a(mul_in_a),
        .mul_in_b(mul_in_b),
        .mul_out(mul_tmp),
        .mul_overflow(mul_overflow)
    );

    ADD #(
        .pDATA_WIDTH(pDATA_WIDTH)
    )add(
        .add_in_a(calc_tmp),
        .add_in_b(mul_tmp),
        .add_out(calc_tmp_w),
        .add_overflow(add_overflow)
    );
/*----------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------*/
    //--STATE-------------------------------------------------------------------------------
    always @(*) begin
        case (state)
            S_IDLE   :   next_state = (ap_start_w) ? S_START : S_IDLE;
            S_START  :   next_state = S_CALCP;
            S_CALCP  :   next_state = (counter==Tape_Num-2)? (((ss_tvalid && ss_tready_reg) || x_in_check) ? S_CALCN  : S_SWAIT) : S_CALCP;
            S_CALCN  :   next_state = ((sm_tready && sm_tvalid_reg) || y_out_check) ? ((x_last) ? S_FINISH : S_CALCP) : S_MWAIT;
            S_SWAIT  :   next_state = (ss_tvalid && ss_tready_reg) ? S_CALCN  : S_SWAIT;
            S_MWAIT  :   next_state = (sm_tready && sm_tvalid_reg) ? ((x_last) ? S_FINISH : S_CALCP) : S_MWAIT;
            S_FINISH :   next_state = (sm_tready && sm_tvalid_reg) ? S_IDLE : S_FINISH;
            default  :   next_state = S_IDLE; 
        endcase
    end
    //--AP_CTRL & data_length & tap_write---------------------------------------------------
    always @(*) begin
        tap_setting_check[0] = 1;
        for ( i=0; i<Tape_Num; i=i+1) begin
            tap_setting_check[i+1] = tap_setting_check[i] & tap_setting[i];
        end
        
        if (axi_lite_addr == {pADDR_WIDTH{1'b0}} && state == S_IDLE) begin
            ap_start_w = (ap_idle && data_length_setting && tap_setting_check[Tape_Num]) ? axi_lite_data[0] : ap_start;
        end
        else if (state == S_START) begin
            ap_start_w = 0;
        end
        else begin
            ap_start_w = ap_start;
        end

        if (axi_lite_addr == {pADDR_WIDTH{1'b0}} && state == S_IDLE) begin
            ap_idle_w = (axi_lite_data[0] && data_length_setting && tap_setting_check[Tape_Num]) ? 0 : ap_idle;
        end
        else if (next_state == S_FINISH) begin
            ap_idle_w = 1;
        end
        else begin
            ap_idle_w = ap_idle;
        end

        if (r_ap_en && state == S_IDLE) begin
            ap_done_w = 0;
        end
        else if (next_state == S_FINISH) begin
            ap_done_w = 1;
        end
        else begin
            ap_done_w = ap_done;
        end

        if (axi_lite_addr == 12'h010 && state == S_IDLE) begin
            data_length_w = axi_lite_data;
            data_length_setting_w = 1;
        end
        else begin
            data_length_w           = data_length;
            data_length_setting_w   = data_length_setting;
        end
    end
    //--TAP_RAM-----------------------------------------------------------------------------
    always @(*) begin
        if ((axi_lite_addr >= 12'h020) && (axi_lite_addr <= 12'h0FF) && state == S_IDLE) begin
            tap_Di_wire = axi_lite_data;
            tap_A_wire  = axi_lite_addr-12'h020;
            tap_WE_wire = 4'b1111;
            tap_EN_wire = 1;
            for (i=0; i<Tape_Num; i=i+1) begin
                if ((axi_lite_addr) == ((12'h020) + (i<<2))) begin
                    tap_setting_w[i] = 1;
                end
            end
        end
        else if (r_tap_valid && state == S_IDLE) begin
            tap_Di_wire = 0;
            tap_A_wire  = r_tap_addr-12'h020;
            tap_WE_wire = 4'b0000;
            tap_EN_wire = 1;
            for ( i=0; i<Tape_Num; i=i+1) begin
                tap_setting_w[i] = tap_setting[i];
            end
        end
        else begin
            tap_Di_wire             = 0;
            tap_A_wire              = (tap_iterator << 2);
            tap_WE_wire             = 4'b0000;
            // tap_EN_wire             = (next_state!=S_IDLE && next_state!=S_FINISH);
            tap_EN_wire             = (next_state!=S_FINISH);
            for ( i=0; i<Tape_Num; i=i+1) begin
                tap_setting_w[i] = tap_setting[i];
            end
        end
    end
    //--DATA_RAM----------------------------------------------------------------------------
    always @(*) begin   
        data_Di_wire = x_in;
        data_EN_wire = (next_state!=S_IDLE && next_state!=S_FINISH && (data_iterator < data_counter));
        data_A_wire  = (data_iterator << 2);
        if (counter == Tape_Num-3) begin
            data_WE_wire = 4'b1111;
        end
        else begin
            data_WE_wire = 4'b0000;
        end
    end
/*----------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------*/
    always @(posedge axis_clk) begin
        if (~axis_rst_n) begin
            //--STATE-----------------------------------------------------------------------
            state <= S_IDLE;
            //--TAP_RAM---------------------------------------------------------------------
            tap_WE_reg  <= 0;
            tap_EN_reg  <= 0;
            tap_Di_reg  <= 0;
            tap_A_reg   <= 0;
            //--DATA_RAM--------------------------------------------------------------------
            data_WE_reg  <= 0;
            data_EN_reg  <= 0;
            data_Di_reg  <= 0;
            data_A_reg   <= 0;
            //--AP_CTRL & data_length & tap_write-------------------------------------------
            ap_start <= 0;
            ap_done  <= 0;
            ap_idle  <= 1;
            data_length <= 0;
            data_length_setting <= 0;
            for ( i=0; i<Tape_Num; i=i+1) begin
                tap_setting[i] <= 0;
            end
            //--AXIS------------------------------------------------------------------------
            ss_tready_reg <= 0;
            sm_tvalid_reg <= 0;
            sm_tdata_reg  <= 0;
            sm_tlast_reg  <= 0;
            //--COUNTER---------------------------------------------------------------------
            data_counter <= 0;
            counter <= 0;
            data_iterator <= 1;
            tap_iterator <= Tape_Num-1;
            //--X_IN------------------------------------------------------------------------
            x_in <= 0;
            x_last <= 0;
            x_in_check <= 0;
            //--Y_OUT-----------------------------------------------------------------------
            y_out_check <= 1;
            //--ALU-------------------------------------------------------------------------
            calc_tmp <= 0;
        end
        else begin
            //--STATE-----------------------------------------------------------------------
            state <= next_state;
            //--TAP_RAM---------------------------------------------------------------------
            tap_WE_reg <= tap_WE_wire;
            tap_EN_reg <= tap_EN_wire;
            tap_Di_reg <= tap_Di_wire;
            tap_A_reg  <= tap_A_wire;
            //--DATA_RAM--------------------------------------------------------------------
            data_WE_reg <= data_WE_wire;
            data_EN_reg <= data_EN_wire;
            data_Di_reg <= data_Di_wire;
            data_A_reg  <= data_A_wire;
            //--AP_CTRL & data_length & tap_write-------------------------------------------
            ap_start <= ap_start_w;
            ap_done  <= ap_done_w;
            ap_idle  <= ap_idle_w;
            data_length <= data_length_w;

            data_length_setting <= (state == S_FINISH)? 0 : data_length_setting_w;
            for ( i=0; i<Tape_Num; i=i+1) begin
                tap_setting[i] <= (state == S_FINISH)? 0 : tap_setting_w[i];
            end
            //--AXIS------------------------------------------------------------------------
            // ss_tready_reg <= (next_state==S_CALCP || next_state==S_SWAIT) && ~ss_tvalid && ~x_in_check;
            ss_tready_reg <= (next_state==S_CALCP || next_state==S_SWAIT) && ~(x_in_check || (ss_tvalid && ss_tready_reg));

            // sm_tvalid_reg <= (next_state==S_CALCP || next_state==S_SWAIT || next_state==S_CALCN) && ~sm_tready && ~y_out_check;
            sm_tvalid_reg <= (next_state==S_CALCP || next_state==S_SWAIT || next_state==S_CALCN || next_state==S_FINISH) && ~(y_out_check || (sm_tready && sm_tvalid_reg));
            if (state==S_CALCN && next_state != S_MWAIT) begin
                sm_tdata_reg <= calc_tmp_w;
            end
            else if (state==S_MWAIT) begin
                sm_tdata_reg <= calc_tmp;
            end
            sm_tlast_reg  <= x_last;
            //--X_IN------------------------------------------------------------------------
            if (ss_tready_reg && ss_tvalid) begin
                x_in    <= ss_tdata;
                x_last  <= ss_tlast;
            end
            else if (next_state==S_IDLE) begin
                x_in    <= 0;
                x_last  <= 0;
            end

            if (next_state==S_CALCN) begin
                x_in_check <= 0;
            end
            else if (ss_tvalid && ss_tready_reg) begin
                x_in_check <= 1;
            end
            //--Y_OUT-----------------------------------------------------------------------
            if (state==S_CALCN) begin
                y_out_check <= 0;
            end
            else if (sm_tready && sm_tvalid_reg) begin
                y_out_check <= 1;
            end
            //--COUNTER--------------------------------------------------------------------
            //----------data_counter-------------------------------------------------------
            if (next_state==S_CALCN) begin
                data_counter <= data_counter + 1;
            end
            else if (next_state == S_FINISH) begin
                data_counter <= 0;
            end
            else begin
                data_counter <= data_counter;
            end
            //----------counter------------------------------------------------------------
            if (next_state==S_CALCP && state==S_CALCP) begin
                counter <= counter + 1;
            end
            else if (next_state==S_CALCN) begin
                counter <= 0;
            end
            else begin
                counter <= counter;
            end
            //----------data_iterator------------------------------------------------------
            if ((next_state==S_START || next_state==S_CALCP || next_state==S_CALCN) && counter==Tape_Num-3) begin
                if (data_iterator==Tape_Num-2) begin
                    data_iterator <= 0;
                end
                else if (data_iterator==Tape_Num-1) begin
                    data_iterator <= 1;
                end
                else begin
                    data_iterator <= data_iterator + 2;
                end
            end
            else if ((next_state==S_START || next_state==S_CALCP || next_state==S_CALCN) && counter!=Tape_Num-3) begin
                if (data_iterator==Tape_Num-1) begin
                    data_iterator <= 0;
                end
                else begin
                    data_iterator <= data_iterator + 1;
                end
            end
            else if (next_state==S_FINISH) begin
                data_iterator <= 1;
            end
            else begin
                data_iterator <= data_iterator;
            end
            //----------tap_iterator-------------------------------------------------------
            if (next_state==S_START || next_state==S_CALCP || next_state==S_CALCN) begin
                if (tap_iterator==0) begin
                    tap_iterator <= Tape_Num - 1;
                end
                else begin
                    tap_iterator <= tap_iterator - 1;
                end
            end
            else if (next_state==S_FINISH) begin
                tap_iterator <= Tape_Num - 1;
            end
            else begin
                tap_iterator <= tap_iterator;
            end
            //--ALU-------------------------------------------------------------------------
            if (state==S_CALCP || (state==S_CALCN && next_state==S_MWAIT)) begin
                calc_tmp <= calc_tmp_w;
            end
            else if ((state==S_CALCN && next_state!=S_MWAIT) || (state==S_MWAIT && next_state!=S_MWAIT)) begin
                calc_tmp <= 0;
            end
        end
    end

endmodule

module S_AXI_LITE #(
    parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32
) (
    output  wire                     awready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,

    output  wire                     wready,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,

    output  wire                     arready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,

    input   wire                     rready,
    output  wire                     rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata,

    output  wire [(pDATA_WIDTH-1):0] axi_lite_data,
    output  wire [(pADDR_WIDTH-1):0] axi_lite_addr,
    input   wire [2:0]               r_ap_data,
    output  wire                     r_tap_valid,
    input   wire [pDATA_WIDTH-1:0]   r_tap_data,
    output  wire [pADDR_WIDTH-1:0]   r_tap_addr,
    output  wire                     r_ap_en,
    input   wire [pDATA_WIDTH-1:0]   r_data_length,
    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);
    localparam AXIR_IDLE = 3'b000;
    localparam AXIR_RA   = 3'b001;
    localparam AXIR_RR   = 3'b010;
    localparam AXIR_RD   = 3'b011;
    localparam AXIR_W1   = 3'b100;
    localparam AXIR_W2   = 3'b101;
    localparam AXIR_W3   = 3'b110;

    localparam AXIW_IDLE = 2'b00;
    localparam AXIW_WA   = 2'b01;
    localparam AXIW_WV   = 2'b10;
    localparam AXIW_WD   = 2'b11;
    
    integer i;
    
    reg [1:0]               axiw_state;
    reg [1:0]               axiw_next_state;

    reg [2:0]               axir_state;
    reg [2:0]               axir_next_state;

    reg                     write_internal_valid;
    reg                     r_tap_valid_reg;
    reg [pADDR_WIDTH-1:0]   r_tap_addr_reg;
    reg [pDATA_WIDTH-1:0]   r_tap_data_reg;
    
    /*axi-lite read register*/
    reg [pADDR_WIDTH-1:0]   araddr_reg;
    reg                     arready_reg;
    reg                     rvalid_reg;
    reg [pDATA_WIDTH-1:0]   rdata_reg;

    /*axi-lite write register*/
    reg [pADDR_WIDTH-1:0]   awaddr_reg;
    reg                     awready_reg;
    reg                     wready_reg;
    reg [pDATA_WIDTH-1:0]   wdata_reg;

    

    /*axi-lite read register*/
    assign arready = arready_reg;
    assign rvalid  = rvalid_reg;
    assign rdata   = rdata_reg;

    /*axi-lite write register*/
    assign awready = awready_reg;
    assign wready  = wready_reg;

    /*axi-lite user wire*/
    assign axi_lite_data = (write_internal_valid) ? wdata_reg  : 0;
    assign axi_lite_addr = (write_internal_valid) ? awaddr_reg : {(pADDR_WIDTH){1'b1}};


    assign r_tap_valid  = r_tap_valid_reg;
    assign r_tap_addr   = r_tap_addr_reg;
    assign r_ap_en      = (axir_state == AXIR_RD) && (araddr_reg == 12'h000);


    always @(*) begin
        case (axiw_state)
            AXIW_IDLE:  axiw_next_state = (awvalid)? AXIW_WA : AXIW_IDLE;
            AXIW_WA:    axiw_next_state = AXIW_WV;
            AXIW_WV:    axiw_next_state = (wvalid)? AXIW_WD : AXIW_WV;
            AXIW_WD:    axiw_next_state = AXIW_IDLE;
            default:    axiw_next_state = AXIW_IDLE;
        endcase
    end

    always @(posedge axis_clk) begin
        if (~axis_rst_n) begin
            awready_reg <= 0;
            awaddr_reg  <= 0;
        end
        else begin
            awready_reg <= (axiw_next_state == AXIW_WA);
            awaddr_reg  <= (axiw_next_state == AXIW_WA) ? awaddr : awaddr_reg;
        end
    end

    always @(posedge axis_clk) begin
        if (~axis_rst_n) begin
            axiw_state <= AXIW_IDLE;
            wready_reg  <= 0;
            wdata_reg   <= 0;
            write_internal_valid <= 0;
        end
        else begin
            axiw_state <= axiw_next_state;
            wready_reg <= (axiw_next_state == AXIW_WD);
            wdata_reg  <= (axiw_next_state == AXIW_WD) ? wdata : 0;
            write_internal_valid <= (axiw_next_state == AXIW_WD);
        end
    end

    always @(*) begin
        case (axir_state)
            AXIR_IDLE:  axir_next_state = (arvalid)? AXIR_RA : AXIR_IDLE;
            AXIR_RA:    axir_next_state = (araddr_reg >= 12'h020 && araddr_reg <= 12'h0FF) ? AXIR_W1 : AXIR_RR;
            AXIR_RR:    axir_next_state = (rready)? AXIR_RD : AXIR_RR;
            AXIR_RD:    axir_next_state = AXIR_IDLE;
            AXIR_W1:    axir_next_state = AXIR_W2;
            AXIR_W2:    axir_next_state = AXIR_W3;
            AXIR_W3:    axir_next_state = AXIR_RR;
            default:    axir_next_state = AXIR_IDLE;
        endcase
    end
    always @(posedge axis_clk) begin
        if (~axis_rst_n) begin
            arready_reg <= 0;
            araddr_reg  <= {(pADDR_WIDTH){1'b1}};
        end
        else begin
            arready_reg <= (axir_next_state == AXIR_RA);
            araddr_reg  <= (axir_next_state == AXIR_RA) ? araddr : araddr_reg;
        end
    end

    always @(posedge axis_clk) begin
        if (~axis_rst_n) begin
            axir_state <= AXIR_IDLE;
            
            rvalid_reg <= 0;
            rdata_reg  <= 0;
            r_tap_valid_reg <= 0;
            r_tap_addr_reg  <= 0;
            r_tap_data_reg  <= 0;
        end
        else begin
            axir_state <= axir_next_state;
            rvalid_reg <= axir_next_state == AXIR_RD;

            r_tap_addr_reg <=  (axir_next_state == AXIR_W1) ? araddr_reg : 0;
            r_tap_valid_reg <= (axir_next_state == AXIR_W1);
            r_tap_data_reg <= (axir_next_state == AXIR_RR)? r_tap_data : r_tap_data_reg;

            if (axir_next_state == AXIR_RD) begin
                if (araddr_reg >= 12'h020 && araddr_reg <= 12'h0FF) begin
                    rdata_reg <= r_tap_data_reg;
                end
                else if((araddr_reg == 12'h000)) begin
                    rdata_reg <= {{(pDATA_WIDTH-3){1'b0}}, r_ap_data};
                end
                else if (araddr_reg == 12'h010) begin
                    rdata_reg <= r_data_length;
                end
                else begin
                    rdata_reg <= {(pDATA_WIDTH){1'b1}};
                end
            end
            else begin
                rdata_reg <= {(pDATA_WIDTH){1'b1}};
            end
        end
    end
endmodule

module ADD #(
    parameter pDATA_WIDTH = 32
)(
    input  wire [pDATA_WIDTH-1:0] add_in_a,
    input  wire [pDATA_WIDTH-1:0] add_in_b,
    output wire [pDATA_WIDTH-1:0] add_out,
    output wire                   add_overflow
);
    localparam max_value = {1'b0, {(pDATA_WIDTH-1){1'b1}}};
    localparam min_value = {1'b1, {(pDATA_WIDTH-1){1'b0}}};
    
    wire [pDATA_WIDTH:0]    add_in_a_extend;
    wire [pDATA_WIDTH:0]    add_in_b_extend;
    wire [pDATA_WIDTH+1:0]  sum_extend;
    wire                    overflow_add;

    assign add_in_a_extend  = {add_in_a[pDATA_WIDTH-1], add_in_a};
    assign add_in_b_extend  = {add_in_b[pDATA_WIDTH-1], add_in_b};
    assign sum_extend       = $signed(add_in_a_extend) + $signed(add_in_b_extend);
    assign overflow_add     = sum_extend[pDATA_WIDTH] ^ sum_extend[pDATA_WIDTH-1];

    assign add_overflow = overflow_add;
    assign add_out      = (overflow_add)? ((sum_extend[pDATA_WIDTH]) ? min_value : max_value) : sum_extend[pDATA_WIDTH-1:0];
endmodule

module MUL #(
    parameter pDATA_WIDTH = 32
) (
    input  wire [pDATA_WIDTH-1:0] mul_in_a,
    input  wire [pDATA_WIDTH-1:0] mul_in_b,
    output wire [pDATA_WIDTH-1:0] mul_out,
    output wire                   mul_overflow
);
    localparam max_value = {1'b0, {(pDATA_WIDTH-1){1'b1}}};
    localparam min_value = {1'b1, {(pDATA_WIDTH-1){1'b0}}};

    wire [pDATA_WIDTH:0]    mul_in_a_extend;
    wire [pDATA_WIDTH:0]    mul_in_b_extend;
    wire [2*pDATA_WIDTH:0]  mul;
    wire                    overflow_mul;

    assign mul_in_a_extend  = {mul_in_a[pDATA_WIDTH-1], mul_in_a};
    assign mul_in_b_extend  = {mul_in_b[pDATA_WIDTH-1], mul_in_b};
    assign mul = $signed(mul_in_a_extend) * $signed(mul_in_b_extend);
    assign overflow_mul = mul[(2*pDATA_WIDTH):(pDATA_WIDTH-1)] != {(pDATA_WIDTH+2){mul[2*pDATA_WIDTH-2]}};

    assign mul_overflow = overflow_mul;
    assign mul_out      = (overflow_mul)? ((mul[2*pDATA_WIDTH]) ? min_value : max_value) : mul[pDATA_WIDTH-1:0];
endmodule
