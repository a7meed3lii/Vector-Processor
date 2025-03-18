module memory_controller #(
    parameter DATA_WIDTH = 16,
    parameter ADDR_WIDTH = 32
)(
    input  logic                  clk,
    input  logic                  rst_n,
    // External memory interface
    output logic                  mem_read_valid,
    output logic [ADDR_WIDTH-1:0] mem_read_addr,
    input  logic                  mem_read_ready,
    input  logic [DATA_WIDTH-1:0] mem_read_data,
    output logic                  mem_write_valid,
    output logic [ADDR_WIDTH-1:0] mem_write_addr,
    output logic [DATA_WIDTH-1:0] mem_write_data,
    input  logic                  mem_write_ready,
    // DMA interface
    input  logic                  dma_read_req,
    input  logic [ADDR_WIDTH-1:0] dma_read_addr,
    output logic [DATA_WIDTH-1:0] dma_read_data,
    output logic                  dma_read_valid,
    input  logic                  dma_write_req,
    input  logic [ADDR_WIDTH-1:0] dma_write_addr,
    input  logic [DATA_WIDTH-1:0] dma_write_data,
    output logic                  dma_write_ack
);

    typedef enum logic [1:0] {
        IDLE,
        READ_REQ,
        READ_WAIT,
        WRITE_REQ
    } mem_state_t;
    
    mem_state_t current_state, next_state;
    logic received_data;

    // State register
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
            received_data <= 1'b0;
        end else begin
            current_state <= next_state;
            
            // Track when we've received data from memory
            if (current_state == READ_REQ && mem_read_ready) 
                received_data <= 1'b1;
            else if (next_state == IDLE)
                received_data <= 1'b0;
        end
    end

    // Next state logic
    always_comb begin
        next_state = current_state;
        
        case (current_state)
            IDLE: begin
                if (dma_read_req)
                    next_state = READ_REQ;
                else if (dma_write_req)
                    next_state = WRITE_REQ;
            end
            
            READ_REQ: begin
                if (mem_read_ready)
                    next_state = READ_WAIT;
            end
            
            READ_WAIT: begin
                next_state = IDLE;
            end
            
            WRITE_REQ: begin
                if (mem_write_ready)
                    next_state = IDLE;
            end
            
            default: next_state = IDLE;
        endcase
    end

    // Output logic
    always_comb begin
        // Default values
        mem_read_valid = 1'b0;
        mem_read_addr = '0;
        mem_write_valid = 1'b0;
        mem_write_addr = '0;
        mem_write_data = '0;
        dma_read_data = mem_read_data;
        dma_read_valid = 1'b0;
        dma_write_ack = 1'b0;
        
        case (current_state)
            IDLE: begin
                // No memory operations in idle state
                // For faster simulation, immediately acknowledge DMA requests in next cycle
                if (dma_read_req) begin
                    mem_read_valid = 1'b1;
                    mem_read_addr = dma_read_addr;
                end
                if (dma_write_req) begin
                    mem_write_valid = 1'b1;
                    mem_write_addr = dma_write_addr;
                    mem_write_data = dma_write_data;
                end
            end
            
            READ_REQ: begin
                mem_read_valid = 1'b1;
                mem_read_addr = dma_read_addr;
            end
            
            READ_WAIT: begin
                dma_read_valid = 1'b1;
            end
            
            WRITE_REQ: begin
                mem_write_valid = 1'b1;
                mem_write_addr = dma_write_addr;
                mem_write_data = dma_write_data;
                dma_write_ack = mem_write_ready;
            end
            
            default: begin
                // Default state - all signals at default values
            end
        endcase
    end

endmodule
