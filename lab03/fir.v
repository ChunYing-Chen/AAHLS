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
    localparam S_FETCH  = 3'b001;
    localparam S_CALC   = 3'b010;
    localparam S_OUT    = 3'b011;
    localparam S_FINISH = 3'b100;
    
    integer                 i;
    
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

    reg  [1:0]              state;
    reg  [1:0]              next_state;
    wire                    done;
    wire                    sent;
    reg                     received;
 
    //--------------------------------------------------------------------------------------
    reg  [3:0]              tap_WE_reg;
    reg                     tap_EN_reg;
    reg  [pDATA_WIDTH-1:0]  tap_Di_reg;
    reg  [pADDR_WIDTH-1:0]  tap_A_reg;

    reg  [3:0]              tap_WE_wire;
    reg                     tap_EN_wire;
    reg  [pDATA_WIDTH-1:0]  tap_Di_wire;
    reg  [pADDR_WIDTH-1:0]  tap_A_wire;
    //--------------------------------------------------------------------------------------
    reg  [3:0]              data_WE_reg;
    reg                     data_EN_reg;
    reg  [pDATA_WIDTH-1:0]  data_Di_reg;
    reg  [pADDR_WIDTH-1:0]  data_A_reg;

    reg  [3:0]              data_WE_wire;
    reg                     data_EN_wire;
    reg  [pDATA_WIDTH-1:0]  data_Di_wire;
    reg  [pADDR_WIDTH-1:0]  data_A_wire;
    //--------------------------------------------------------------------------------------

    wire [pDATA_WIDTH-1:0]  axi_lite_data;
    wire [pADDR_WIDTH-1:0]  axi_lite_addr;

    reg                     ss_tready_reg;
    reg                     sm_tvalid_reg;
    reg [(pDATA_WIDTH-1):0] sm_tdata_reg;
    reg                     sm_tlast_reg;

    reg                     ss_tready_w;

    reg  [$clog2(Tape_Num)-1:0] counter;
    reg  [$clog2(Tape_Num)-1:0] iterator;
    reg  [pDATA_WIDTH-1:0]  x_in;
    reg                     x_last;

    wire [pDATA_WIDTH-1:0]  mul_in_a;
    wire [pDATA_WIDTH-1:0]  mul_in_b;
    wire [pDATA_WIDTH-1:0]  mul_tmp;
    reg  [pDATA_WIDTH-1:0]  calc_tmp;
    wire  [pDATA_WIDTH-1:0] calc_tmp_w;
    wire                    mul_new_data;

    wire                    r_ap_en;
    wire                    r_tap_valid;
    wire [pADDR_WIDTH-1:0]  r_tap_addr;
    reg                     r_tap_data_en;

    reg                     tap_setting_check   [Tape_Num:0];
    //-------unused!---------------------
    wire                    mul_overflow;
    wire                    add_overflow;
    //-----------------------------------

    assign tap_WE = tap_WE_reg;
    assign tap_EN = tap_EN_reg;
    assign tap_Di = tap_Di_reg;
    assign tap_A  = tap_A_reg;

    assign data_WE = data_WE_reg;
    assign data_EN = data_EN_reg;
    assign data_Di = data_Di_reg;
    assign data_A  = data_A_reg;

    assign ss_tready = ss_tready_reg;
    assign sm_tvalid = sm_tvalid_reg;
    assign sm_tlast  = sm_tlast_reg;
    assign sm_tdata  = sm_tdata_reg;

    assign mul_in_a = (mul_new_data)? ((state == S_CALC) ? x_in : 0) : data_Do;
    assign mul_in_b = tap_Do;
    assign mul_new_data = (counter == iterator);
    assign done = (counter == iterator) && (state == S_CALC);
    assign sent = sm_tready;

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
        .r_tap_data(tap_Do),
        .r_tap_addr(r_tap_addr),
        .r_ap_en(r_ap_en),
        .r_data_length(data_length),
        .r_tap_data_en(r_tap_data_en),
        .axis_clk(axis_clk),
        .axis_rst_n(axis_rst_n),
        .next_state(next_state)
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

    always @(*) begin
        case (state)
            S_IDLE:     next_state = (ap_start_w)   ? S_FETCH : S_IDLE;
            S_FETCH:    next_state = (received)     ? S_CALC  : S_FETCH;
            S_CALC:     next_state = (done)         ? S_OUT   : S_CALC;
            S_OUT:      next_state = (sent)         ? ((x_last) ? S_FINISH : S_IDLE) : S_OUT;
            S_FINISH:   next_state = S_IDLE;
            default:    next_state = S_IDLE; 
        endcase
    end

    always @(*) begin
        tap_setting_check[0] = 1;
        for ( i=0; i<Tape_Num; i=i+1) begin
            tap_setting_check[i+1] = tap_setting_check[i] & tap_setting[i];
        end
        
        if (axi_lite_addr == {pADDR_WIDTH{1'b0}} && state == S_IDLE) begin
            ap_start_w = (ap_idle && data_length_setting && tap_setting_check[Tape_Num]) ? axi_lite_data[0] : ap_start;
        end
        else if (state == S_FETCH) begin
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

        if (r_ap_en) begin
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
            r_tap_data_en = 0;
        end
        else if (r_tap_valid && state == S_IDLE) begin
            tap_Di_wire = 0;
            tap_A_wire  = r_tap_addr-12'h020;
            tap_WE_wire = 4'b0000;
            tap_EN_wire = 1;
            for ( i=0; i<Tape_Num; i=i+1) begin
                tap_setting_w[i] = tap_setting[i];
            end
            r_tap_data_en = 1;
        end
        else begin
            tap_Di_wire             = 0;
            tap_A_wire              = (11-iterator)<<2;
            tap_WE_wire             = 4'b0000;
            tap_EN_wire             = 1;
            for ( i=0; i<Tape_Num; i=i+1) begin
                tap_setting_w[i] = tap_setting[i];
            end
            r_tap_data_en = 0;
        end

    end

    always @(*) begin
        ss_tready_w  = (next_state == S_FETCH);
        received = (state == S_FETCH && ss_tvalid);
        if (next_state != S_OUT) begin
            data_Di_wire = 0;
            data_EN_wire = 1;
            data_WE_wire = 4'b0000;
            data_A_wire  = (iterator << 2);
        end
        else begin
            data_Di_wire = x_in;
            data_EN_wire = 1;
            data_WE_wire = 4'b1111;
            data_A_wire  = (counter << 2);
        end

    end

    always @(posedge axis_clk) begin
        if (~axis_rst_n) begin
            tap_WE_reg  <= 0;
            tap_EN_reg  <= 0;
            tap_Di_reg  <= 0;
            tap_A_reg   <= 0;

            data_WE_reg  <= 0;
            data_EN_reg  <= 0;
            data_Di_reg  <= 0;
            data_A_reg   <= 0;

            ap_start <= 0;
            ap_done  <= 0;
            ap_idle  <= 1;
            data_length <= 0;

            data_length_setting <= 0;
            for ( i=0; i<Tape_Num; i=i+1) begin
                tap_setting[i] <= 0;
            end

            state <= S_IDLE;

            ss_tready_reg <= 0;
            sm_tvalid_reg <= 0;
            sm_tdata_reg  <= 0;
            sm_tlast_reg  <= 0;

            counter <= 0;
            iterator <= 1;
            x_in <= 0;
            x_last <= 0;

            calc_tmp <= 0;
        end
        else begin
            state <= next_state;

            tap_WE_reg <= tap_WE_wire;
            tap_EN_reg <= tap_EN_wire;
            tap_Di_reg <= tap_Di_wire;
            tap_A_reg  <= tap_A_wire;

            data_WE_reg <= data_WE_wire;
            data_EN_reg <= data_EN_wire;
            data_Di_reg <= data_Di_wire;
            data_A_reg  <= data_A_wire;

            ap_start <= ap_start_w;
            ap_done  <= ap_done_w;
            ap_idle  <= ap_idle_w;
            data_length <= data_length_w;

            data_length_setting <= (state == S_FINISH)? 0 : data_length_setting_w;
            for ( i=0; i<Tape_Num; i=i+1) begin
                tap_setting[i] <= (state == S_FINISH)? 0 : tap_setting_w[i];
            end

            ss_tready_reg <= ss_tready_w;

            if (state == S_FETCH && ss_tvalid) begin
                x_in   <= ss_tdata;
                x_last <= ss_tlast;
            end

            if (state == S_OUT) begin
                if (counter == Tape_Num-1) begin
                    counter <= 0;
                end
                else begin
                    counter <= counter + 1;
                end
            end

            if (state == S_OUT) begin
                if (counter == Tape_Num-2) begin
                    iterator <= 0;
                end
                else if (counter == Tape_Num-1) begin
                    iterator <= 1;
                end
                else begin
                    iterator <= counter + 2;
                end
            end
            else if (next_state == S_CALC || next_state == S_FETCH) begin
                if ((iterator == Tape_Num-1) && (counter != Tape_Num-1)) begin
                    iterator <= 0;
                end
                else if(counter != iterator) begin
                    iterator <= iterator + 1;
                end
                else begin
                    iterator <= iterator;
                end
            end
            
            calc_tmp <= (state == S_FETCH || state == S_CALC) ? ((done) ? 0 : calc_tmp_w) : calc_tmp;

            sm_tvalid_reg <= done || (state == S_OUT && ~sm_tready);
            sm_tdata_reg  <= (done) ? calc_tmp_w : ((next_state == S_OUT) ? sm_tdata_reg : 0);
            sm_tlast_reg  <= (next_state == S_OUT) ? x_last : 0;

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
    input   wire                     r_tap_data_en,
    input   wire                     axis_clk,
    input   wire                     axis_rst_n,
    input   wire [2:0]               next_state
);
    localparam AXIR_IDLE = 3'b000;
    localparam AXIR_RA   = 3'b001;
    localparam AXIR_RD   = 3'b010;
    localparam AXIR_W1   = 3'b011;
    localparam AXIR_W2   = 3'b100;
    localparam AXIR_W3   = 3'b101;
    
    integer i;
    
    reg [2:0]               axir_state;
    reg [2:0]               axir_next_state;

    reg                     write_internal_valid;
    reg                     r_tap_valid_reg;
    reg [pADDR_WIDTH-1:0]   r_tap_addr_reg;
    
    /*axi-lite read register*/
    reg [pADDR_WIDTH-1:0]   araddr_reg;
    reg                     arready_reg;
    reg                     rvalid_reg;
    reg [pDATA_WIDTH-1:0]   rdata_reg;
    reg                     r_tap_data_en_shift;
    reg                     r_tap_data_en_shift2;

    /*axi-lite write register*/
    reg [pADDR_WIDTH-1:0]   awaddr_reg;
    reg                     awready_reg;
    reg                     wready_reg;
    reg [pDATA_WIDTH-1:0]   wdata_reg;
    reg [pDATA_WIDTH-1:0]   r_tap_data_reg;

    

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
    assign r_ap_en      = ~rvalid_reg && rready && (araddr_reg == 12'h000);

    always @(posedge axis_clk) begin
        if (~axis_rst_n) begin
            awready_reg <= 0;
            awaddr_reg  <= 0;
        end
        else begin
            awready_reg <= (awvalid && ~awready_reg);
            awaddr_reg  <= (awvalid && ~awready_reg) ? awaddr : awaddr_reg;
        end
    end

    always @(posedge axis_clk) begin
        if (~axis_rst_n) begin
            wready_reg  <= 0;
            wdata_reg   <= 0;
            write_internal_valid <= 0;
        end
        else begin
            wready_reg <= (wvalid && ~wready_reg && awready_reg);
            if (wvalid && ~wready_reg) begin
                wdata_reg <= wdata;
                write_internal_valid <= 1;
            end
            else begin
                write_internal_valid <= 0;
            end
        end
    end

    always @(*) begin
        case (axir_state)
            AXIR_IDLE:  axir_next_state = (arvalid)? AXIR_RA : AXIR_IDLE;
            AXIR_RA:    axir_next_state = (araddr_reg >= 12'h020 && araddr_reg <= 12'h0FF) ? AXIR_W1 : AXIR_RD;
            AXIR_RD:    axir_next_state = AXIR_IDLE;
            AXIR_W1:    axir_next_state = AXIR_W2;
            AXIR_W2:    axir_next_state = AXIR_W3;
            AXIR_W3:    axir_next_state = AXIR_RD;
            default:    axir_next_state = AXIR_IDLE;
        endcase
    end
    always @(posedge axis_clk) begin
        if (~axis_rst_n) begin
            arready_reg <= 0;
            araddr_reg  <= 0;
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
        end
        else begin
            axir_state <= axir_next_state;
            rvalid_reg <= axir_next_state == AXIR_RD;

            r_tap_addr_reg <=  (axir_next_state == AXIR_W1) ? araddr_reg : 0;
            r_tap_valid_reg <= (axir_next_state == AXIR_W1);

            if (axir_next_state == AXIR_RD) begin
                if (araddr_reg >= 12'h020 && araddr_reg <= 12'h0FF) begin
                    rdata_reg <= r_tap_data;
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
                rdata_reg <= 0;
            end
        end
    end
endmodule

module ADD #(
    pDATA_WIDTH = 32
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
    pDATA_WIDTH = 32
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

    assign mul = $signed(mul_in_a_extend) * $signed(mul_in_b_extend);
    assign overflow_mul = mul[(2*pDATA_WIDTH):(pDATA_WIDTH-1)] != {(pDATA_WIDTH+2){mul[2*pDATA_WIDTH-2]}};

    assign mul_overflow = overflow_mul;
    assign mul_out      = (overflow_mul)? ((mul[2*pDATA_WIDTH]) ? min_value : max_value) : mul[pDATA_WIDTH-1:0];
endmodule