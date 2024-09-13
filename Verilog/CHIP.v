

module CHIP(clk,
            rst_n,
            // For mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // For mem_I
            mem_addr_I,
            mem_rdata_I);

    input         clk, rst_n ;
    // For mem_D
    output reg    mem_wen_D  ; // 0: Read data from data/stack memory, 1: Write data to data/stack memory
    output reg [31:0] mem_addr_D ; // Address of data/stack memory
    output reg [31:0] mem_wdata_D; // Data written to data/stack memory
    input  [31:0] mem_rdata_D; // Data read from data/stack memory
    // For mem_I
    output reg [31:0] mem_addr_I ; // Address of instruction (text) memory
    input  [31:0] mem_rdata_I; // Instruction read from instruction (text) memory
    
    
    //---------------------------------------//
    // Do not modify this part!!!            //
    // Exception: You may change wire to reg //
    reg    [31:0] PC          ;              //
    reg    [31:0] PC_nxt      ;              //
    reg           regWrite    ;              //
    reg    [ 4:0] rs1, rs2, rd;              //
    wire   [31:0] rs1_data    ;              //
    wire   [31:0] rs2_data    ;              //
    wire   [31:0] rd_data     ;              //
    //---------------------------------------//

    // Todo: other wire/reg
    reg [31:0] Instruction_all ; // instruction code
    reg [6:0] opcode ; // Instruction_all[6:0]
    reg [31:0] Read_data ; // data read from data/stack memory
    reg [31:0] Write_data ; // write data to data memory
    reg [31:0] Immediate; // Immediate = Instruction_all[31:0]

    wire [31:0] Imm_Gen; // expand Immediate
    wire [3:0] control ; // control signal
    wire [31:0] ALU_result ; // result from ALU
    wire [31:0] ALU_Input2 ; // ALU input_2
    wire [31:0] mux_output; // mux after data memory

    // control signal for different blocks
    wire Zero;
    reg Jalr, Jal, Branch, MemRead, MemtoReg, MemWrite, ALUSrc, Auipc, JalJalr;
    reg [1:0] jal_jalr_auipc;
    reg [2:0] ALUOp ;

    // PC signal
    reg [31:0] PCAdd4; // PC add 4
    reg [31:0] PC_for_branch; // PC for branch
    reg [31:0] PC_for_jal; // PC for jump
    reg [31:0] PC_for_jalr; // PC for jump and link register
    wire [63:0] muldiv_result; // result from muldiv
    wire muldiv_valid; // start muldiv
    wire muldiv_ready; // result is ready
    wire muldiv_mode; // for mul or div

    //---------------------------------------//
    // Do not modify this part!!!            //
    reg_file reg0(                           //
        .clk(clk),                           //
        .rst_n(rst_n),                       //
        .wen(regWrite),                      //
        .a1(rs1),                            //
        .a2(rs2),                            //
        .aw(rd),                             //
        .d(rd_data),                         //
        .q1(rs1_data),                       //
        .q2(rs2_data));                      //
    //---------------------------------------//
    
    // Todo: any combinational/sequential circuit

    ALU ALU_instance(.Input1(rs1_data), .Input2(ALU_Input2), .result(ALU_result), .Zero(Zero), .control(control), .muldiv_valid(muldiv_valid), .muldiv_ready(muldiv_ready), .muldiv_result(muldiv_result));

    ALU_control ALU_control_instance(.ALUOp(ALUOp), .bit_30_of_instruction(Immediate[30]), .bit_25_of_instruction(Immediate[25]), .function3(Immediate[14:12]), .control(control));
    
    Imm_Gen_block Imm_Gen_instance(.Imm_Gen(Imm_Gen), .Immediate(Immediate));

    mulDiv mulDiv_instance(.clk(clk), .rst_n(rst_n), .valid(muldiv_valid), .ready(muldiv_ready), .mode(muldiv_mode), .in_A(rs1_data), .in_B(ALU_Input2), .out(muldiv_result));

    Mux mux_before_ALU_instance(.Input0(rs2_data), .Input1(Imm_Gen), .Output(ALU_Input2), .control(ALUSrc));

    Mux mux_after_Data_memory(.Input0(ALU_result), .Input1(Read_data), .Output(mux_output), .control(MemtoReg));

    Mux_four_to_one mux_final(.Input00(mux_output), .Input01(PC_for_branch), .Input10(PCAdd4), .Input11(PC_for_branch), .Output(rd_data), .control1(JalJalr), .control2(Auipc));
    
    assign muldiv_mode = 1'b0 ;

    always@(*)
    begin
        Instruction_all = mem_rdata_I;
        Read_data = mem_rdata_D ;

        opcode = Instruction_all[6:0];
        rs1 = Instruction_all[19:15];
        rs2 = Instruction_all[24:20];
        rd = Instruction_all[11:7];
        Immediate = Instruction_all[31:0];
        
        case(opcode)
        7'b0000011: //lw
        begin
            Branch = 0;
            MemRead = 1;
            MemtoReg = 1;
            MemWrite = 0;
            regWrite = 1;
            ALUOp = 3'b010;
            ALUSrc = 1;
            Auipc = 0;
            JalJalr = 0;
            Jal = 0;
            Jalr = 0;
            jal_jalr_auipc=2'b00;

        end
        7'b0010011: //addi, slti, srli
        begin

            Branch = 0;
            MemRead = 0;
            MemtoReg = 0;
            MemWrite = 0;
            regWrite = 1;
            ALUOp = 3'b001; 
            ALUSrc = 1;
            Auipc = 0;
            JalJalr = 0;
            Jal = 0;
            Jalr = 0;
            jal_jalr_auipc=2'b00;
        end
        7'b0010111: //auipc  
        begin

            Branch = 0;
            MemRead = 0;
            MemtoReg = 0;
            MemWrite = 0;
            regWrite = 1;
            ALUOp = 3'b111;
            ALUSrc = 1;
            Auipc = 1;
            JalJalr = 0;
            Jal = 0;
            Jalr = 0;
            jal_jalr_auipc=2'b01;
        end
        7'b0100011: //sw
        begin

            Branch = 0;
            MemRead = 0;
            MemtoReg = 0;
            MemWrite = 1;
            regWrite = 0;
            ALUOp = 3'b011;
            ALUSrc = 1;
            Auipc = 0;
            JalJalr = 0;
            Jal = 0;
            Jalr = 0;
            jal_jalr_auipc=2'b00;
        end
        7'b0110011: //and, sub, add, or, slt, mul
        begin

            Branch = 0;
            MemRead = 0;
            MemtoReg = 0;
            MemWrite = 0;
            if (muldiv_valid == 1 && muldiv_ready == 0) 
            begin
                regWrite = 0;
            end
            else 
            begin
                regWrite = 1;
            end
            ALUOp = 3'b000; 
            ALUSrc = 0;
            Auipc = 0;
            JalJalr = 0;
            Jalr = 0;
            Jal = 0;
            jal_jalr_auipc=2'b00;
        end
        7'b1100011: //beq
        begin

            Branch = 1;
            MemRead = 0;
            MemtoReg = 0;
            MemWrite = 0;
            regWrite = 0;
            ALUOp = 3'b100;
            ALUSrc = 0;
            Auipc = 0;
            JalJalr = 0;
            Jal = 0;
            Jalr = 0;
            jal_jalr_auipc=2'b00;
        end
        7'b1100111: //jalr  
        begin

            Branch = 0;
            MemRead = 0;
            MemtoReg = 0;
            MemWrite = 0;
            regWrite = 1;
            ALUOp = 3'b110;
            ALUSrc = 1;
            Auipc = 0;
            JalJalr = 1;
            Jal = 0;
            Jalr = 1;
            jal_jalr_auipc=2'b10;
        end
        7'b1101111: //jal
        begin

            Branch = 0;
            MemRead = 0;
            MemtoReg = 0;
            MemWrite = 0;
            regWrite = 1;
            ALUOp = 3'b101;
            ALUSrc = 1;
            Auipc = 0;
            JalJalr = 1;
            Jal = 1;
            Jalr = 0;
            jal_jalr_auipc=2'b10;
        end
        default   
        begin

            Branch = 0;
            MemRead = 0;
            MemtoReg = 0;
            MemWrite = 0;
            regWrite = 0;
            ALUOp = 3'b000;
            ALUSrc = 0;
            Auipc = 0;
            JalJalr = 0;
            Jal = 0;
            Jalr = 0;
            jal_jalr_auipc=2'b00;
        end
        endcase

        // determine PC signal
        PCAdd4 <= PC + 4;
        PC_for_branch = PC + Imm_Gen;
        PC_for_jalr = Imm_Gen + rs1_data;
        if ((((Branch & Zero)==1)&(Jal==0)) | (((Branch & Zero)==0)&(Jal==1)))
            PC_for_jal = PC_for_branch;
        else
            PC_for_jal = PCAdd4;

        if (rst_n==0)
        	PC_nxt = 0;
        else if (muldiv_valid == 1 && muldiv_ready == 0) 
            PC_nxt = PC;
        else if (Jalr==1)
            PC_nxt = PC_for_jalr;
        else
            PC_nxt = PC_for_jal;



        if (MemWrite)
            Write_data = rs2_data;
        else
            Write_data = 0;

        mem_wdata_D = Write_data;
        mem_wen_D = MemWrite;
        mem_addr_D = ALU_result;
        mem_addr_I = PC;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
            
        end
        else begin
            PC <= PC_nxt;
            
        end
    end
endmodule

module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw;

    output [BITS-1:0] q1, q2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign q1 = mem[a1];
    assign q2 = mem[a2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

// multiplxer for 32 bits
module Mux(Input0, Input1, Output, control);
    input [31:0] Input0;
    input [31:0] Input1;
    input control;
    output [31:0] Output;
    assign Output = control ? Input1 : Input0; // Control == 0 => Input0, else => Input1
endmodule

// multiplxer for 32 bits, 4 to 1.
module Mux_four_to_one(Input00, Input01, Input10, Input11, Output, control1, control2);
    input [31:0] Input00;
    input [31:0] Input01;
    input [31:0] Input10;
    input [31:0] Input11;
    input control1;
    input control2;
    output [31:0] Output;
    assign Output = (control1==0 & control2==0) ?   Input00 : (control1==1 & control2==0) ?   Input10 : (control1==0 & control2==1)? Input01: Input11;
endmodule


module mulDiv(clk, rst_n, valid, ready, mode, in_A, in_B, out);
    // Todo: your HW2
    // Definition of ports
    input         clk, rst_n;
    input         valid, mode; // mode: 0: mulu, 1: divu
    output        ready;
    input  [31:0] in_A, in_B;
    output [63:0] out;

    // Definition of states
    parameter IDLE = 2'b00;
    parameter MUL  = 2'b01;
    parameter DIV  = 2'b10;
    parameter OUT  = 2'b11;

    // Todo: Wire and reg if needed
    reg  [ 1:0] state, state_nxt;
    reg  [ 4:0] counter, counter_nxt;
    reg  [63:0] shreg, shreg_nxt;
    reg  [31:0] alu_in, alu_in_nxt;
    reg  [32:0] alu_out;

    // Todo: Instatiate any primitives if needed

    // Todo 5: Wire assignments
    assign ready = &state; 
    assign out = shreg_nxt;

    // Combinational always block
    // Todo 1: Next-state logic of state machine
    always @(*) begin // implement always block if signal changes
        case(state)
            IDLE: begin
                if (~valid) state_nxt = state; // valid == 0，keep IDLE
                else begin
                    if (mode) state_nxt = DIV; // mode == 1，DIV
                    else state_nxt = MUL; // mode == 0，MUL
                end
            end
            MUL : state_nxt = (counter != 31)?state: OUT; // if counter == 31，OUT
            DIV : state_nxt = (counter != 31)?state: OUT; // if counter == 31，OUT
            OUT : state_nxt = IDLE;
        endcase
    end
    
    // Todo 2: Counter
    always @(*) begin // implement always block if signal changes
        case(state)
            IDLE: counter_nxt = 0;
            MUL : counter_nxt = (counter != 31)?counter + 1: 0; 
            DIV : counter_nxt = (counter != 31)?counter + 1: 0; 
            OUT : counter_nxt = 0;
        endcase  
    end   
    
    // ALU input
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) alu_in_nxt = in_B;
                else       alu_in_nxt = 0;
            end
            OUT : alu_in_nxt = 0;
            default: alu_in_nxt = alu_in;
        endcase
    end

    // Todo 3: ALU output
    always @(*) begin
        case(state)
            IDLE: alu_out = {1'b0, shreg[63:32]};
            MUL : begin
                if (~shreg[0]) alu_out = {1'b0, shreg[63:32]}; // fill msb with 1'b0
                else alu_out = {1'b0, alu_in} + {1'b0, shreg[63:32]}; // fill msb with 1'b0              
            end
            DIV : begin
                alu_out = shreg[63:31] - {1'b0, alu_in};
            end
            OUT : alu_out = {1'b0, shreg[63:32]};
        endcase
    end   
    
    // Todo 4: Shift register
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) shreg_nxt = {32'b0, in_A};
                else shreg_nxt = 64'b0;
            end
            MUL : begin
                shreg_nxt = {alu_out, shreg[31:1]}; // shift right
            end
            DIV : begin
                if (alu_out[32]) shreg_nxt = {shreg[62:0], 1'b0}; // shift left
                else shreg_nxt = {alu_out[31:0], shreg[30:0], 1'b1}; // shift left
            end
            OUT : shreg_nxt = shreg;
        endcase
    end   

    // Todo: Sequential always block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin // rst_n == 0: reset，state = IDLE
            state <= IDLE;
            alu_in  <= 0;
            shreg   <= 0;
            counter <= 0;
        end
        else begin
            state <= state_nxt;
            alu_in  <= alu_in_nxt;
            shreg   <= shreg_nxt;
            counter <= counter_nxt;
        end
    end

endmodule

module ALU(Input1, Input2, result, Zero, control, muldiv_valid, muldiv_ready, muldiv_result);
input [31:0] Input1, Input2;
input [3:0] control ; // ALUControl signal
output reg [31:0] result;
output reg Zero; // equal or not for branch
output reg muldiv_valid;
input  muldiv_ready;
input [63:0] muldiv_result;
always@(*)
begin
    case(control)
        4'b1111: // add, addi, lw, sw
        begin
            result = $signed(Input1) + $signed(Input2); 
            Zero = 0;
            muldiv_valid = 0;
        end
        4'b1001: // srli
        begin
            result = Input1 >> Input2;
            Zero = 0;
           muldiv_valid = 0;
        end
        4'b1101: //slt, and, or, slti
        begin
            if ( $signed(Input1) < $signed(Input2))
                result = 1;
            else 
                result = 0;
            Zero = 0;
            muldiv_valid = 0;
        end
        4'b1100: // mul 
        begin
            if (muldiv_ready == 1)
                result = muldiv_result[31:0];
            else
                result = 0;
            Zero = 0 ;
            muldiv_valid = 1;
        end
        4'b1110: // sub, beq, (jal, jalr, auipc)
        begin
            result = $signed(Input1) - $signed(Input2);
            if (result == 0) // for branch
                Zero = 1;
            else 
                Zero = 0;
            muldiv_valid = 0;
        end
        default 
        begin
            result = 0;
            Zero = 0;
            muldiv_valid = 0;
        end
    endcase
end
endmodule

// ALU control 
module ALU_control(ALUOp, bit_30_of_instruction, bit_25_of_instruction, function3, control);
input bit_30_of_instruction;
input bit_25_of_instruction; 
input [2:0] ALUOp; 
input [2:0] function3;
output reg [3:0] control; // ALU output
always @(*)
begin
    control = 4'b0000; // default value
    case(ALUOp)
    3'b000:
    begin
        case(function3)
        3'b000:
        begin
            case(bit_30_of_instruction)
            1'b0:
            begin
                case(bit_25_of_instruction)
                1'b0:
                begin
                    control = 4'b1111;// add
                end
                1'b1:
                begin
                    control = 4'b1100;// mul
                end
                endcase
            end
            1'b1:
            begin
                control = 4'b1110;// sub
            end
            endcase
        end
        3'b010:
        begin
            control = 4'b1101;// slt
        end
        3'b110:
        begin
            control = 4'b1101;// or
        end
        3'b111:
        begin
            control = 4'b1101;// and
        end
        endcase
    end
    3'b001:
    begin
        case(function3)
        3'b000:
        begin
            control = 4'b1111;// addi
        end
        3'b010:
        begin
            control = 4'b1101;// slti
        end
        3'b101:
        begin
            control = 4'b1001;// srli
        end
        endcase
    end
    3'b010:
    begin
        control = 4'b1111;// lw
    end
    3'b011:
    begin
        control = 4'b1111;// sw
    end 
    3'b100:
    begin
        control = 4'b1110;// beq
    end 
    3'b101:
    begin
        control = 4'b1110;// jal
    end
    3'b110:
    begin
        control = 4'b1110;// jalr
    end
    3'b111:
    begin
        control = 4'b1110;// auipc
    end
    endcase
end
endmodule

// Imm Gen
// immediate extend to 64 bits, only lw, sw, jal, jalr, addi, slti, beq
module Imm_Gen_block(Imm_Gen, Immediate);
input [31:0] Immediate;
output reg [31:0] Imm_Gen ;
reg [6:0] opcode;

always@(*)
begin
    opcode = Immediate[6:0];
    case(opcode)
    7'b0000011: //lw
    begin
        Imm_Gen[31:11] = {21{Immediate[31]}};
        Imm_Gen[10:0] = Immediate[30:20]; 
    end
    7'b0010011: //addi, slti
    begin
        Imm_Gen[31:11] = {21{Immediate[31]}};
        Imm_Gen[10:0] = Immediate[30:20];
    end
    7'b0100011: //sw
    begin
        Imm_Gen[31:11] = {21{Immediate[31]}};
        Imm_Gen[10:5] = Immediate[30:25];
        Imm_Gen[4:0] = Immediate[11:7];
    end
    7'b1100011: //beq
    begin
        Imm_Gen[31:12] = {20{Immediate[31]}};
        Imm_Gen[11] = Immediate[7];
        Imm_Gen[10:5] = Immediate[30:25];
        Imm_Gen[4:1] = Immediate[11:8];
        Imm_Gen[0] = 0 ;
    end
    7'b1100111: //jalr
    begin
        Imm_Gen[31:11] = {21{Immediate[31]}};
        Imm_Gen[10:0] = Immediate[30:20];
    end
    7'b1101111: //jal
    begin
        Imm_Gen[31:20] = {12{Immediate[31]}};
        Imm_Gen[19:12] = Immediate[19:12];
        Imm_Gen[11] = Immediate[20];
        Imm_Gen[10:1] = Immediate[30:21];
        Imm_Gen[0] = 0; 
    end
    default
    begin
        Imm_Gen[31:0] = {32{1'b0}};
    end
    endcase
end
endmodule

