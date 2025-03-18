module vector_register_file #(
    parameter DATA_WIDTH = 16,
    parameter IMG_WIDTH = 224,
    parameter IMG_HEIGHT = 224,
    parameter REGISTER_COUNT = 8  // Number of vector registers
)(
    input  logic                  clk,
    input  logic                  rst_n,
    // Input interface
    input  logic [DATA_WIDTH-1:0] in_data,
    input  logic                  in_valid,
    output logic                  in_ready,
    // Output interface
    output logic [DATA_WIDTH-1:0] out_data,
    output logic                  out_valid,
    input  logic                  out_ready
);

    // Constants
    localparam VECTOR_SIZE = IMG_WIDTH * IMG_HEIGHT / REGISTER_COUNT;
    
    // Vector register file
    logic [DATA_WIDTH-1:0] registers [REGISTER_COUNT-1:0][VECTOR_SIZE-1:0];
    
    // Internal control signals
    logic [$clog2(REGISTER_COUNT)-1:0] write_reg;
    logic [$clog2(VECTOR_SIZE)-1:0] write_idx;
    logic [$clog2(REGISTER_COUNT)-1:0] read_reg;
    logic [$clog2(VECTOR_SIZE)-1:0] read_idx;
    logic write_in_progress;
    logic read_in_progress;
    
    // FSM states
    typedef enum logic [1:0] {
        IDLE,
        WRITING,
        READING,
        FULL
    } reg_state_t;
    
    reg_state_t current_state, next_state;
    
    // State register
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
            write_reg <= '0;
            write_idx <= '0;
            read_reg <= '0;
            read_idx <= '0;
            write_in_progress <= 1'b0;
            read_in_progress <= 1'b0;
        end else begin
            current_state <= next_state;
            
            // Write pointer logic
            if (current_state == WRITING && in_valid) begin
                if (write_idx == VECTOR_SIZE - 1) begin
                    write_idx <= '0;
                    if (write_reg == REGISTER_COUNT - 1)
                        write_reg <= '0;
                    else
                        write_reg <= write_reg + 1;
                end else begin
                    write_idx <= write_idx + 1;
                end
            end
            
            // Read pointer logic
            if (current_state == READING && out_ready) begin
                if (read_idx == VECTOR_SIZE - 1) begin
                    read_idx <= '0;
                    if (read_reg == REGISTER_COUNT - 1)
                        read_reg <= '0;
                    else
                        read_reg <= read_reg + 1;
                end else begin
                    read_idx <= read_idx + 1;
                end
            end
            
            // Track progress
            write_in_progress <= (next_state == WRITING);
            read_in_progress <= (next_state == READING);
        end
    end
    
    // Memory write logic
    always_ff @(posedge clk) begin
        if (current_state == WRITING && in_valid) begin
            registers[write_reg][write_idx] <= in_data;
        end
    end
    
    // Next state logic
    always_comb begin
        next_state = current_state;
        
        case (current_state)
            IDLE: begin
                if (in_valid)
                    next_state = WRITING;
                else if (out_ready)
                    next_state = READING;
            end
            
            WRITING: begin
                if (!in_valid) begin
                    if (out_ready)
                        next_state = READING;
                    else
                        next_state = IDLE;
                end else if (write_reg == REGISTER_COUNT - 1 && write_idx == VECTOR_SIZE - 1) begin
                    next_state = FULL;
                end
            end
            
            READING: begin
                if (!out_ready) begin
                    if (in_valid)
                        next_state = WRITING;
                    else
                        next_state = IDLE;
                end else if (read_reg == REGISTER_COUNT - 1 && read_idx == VECTOR_SIZE - 1) begin
                    next_state = IDLE;
                end
            end
            
            FULL: begin
                if (out_ready)
                    next_state = READING;
            end
            
            default: next_state = IDLE;
        endcase
    end
    
    // Output logic
    assign in_ready = (current_state == IDLE || current_state == WRITING) && (current_state != FULL);
    assign out_data = (current_state == READING) ? registers[read_reg][read_idx] : '0;
    assign out_valid = (current_state == READING);

endmodule
