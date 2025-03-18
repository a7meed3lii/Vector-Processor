module dma_controller #(
    parameter DATA_WIDTH = 16,
    parameter ADDR_WIDTH = 32,
    parameter IMG_WIDTH = 224,
    parameter IMG_HEIGHT = 224,
    parameter IMG_CHANNELS = 1
)(
    input  logic                  clk,
    input  logic                  rst_n,
    // Control signals
    input  logic                  start,
    input  logic [ADDR_WIDTH-1:0] input_addr,
    input  logic [ADDR_WIDTH-1:0] weight_addr,
    input  logic [ADDR_WIDTH-1:0] bn_param_addr,
    input  logic [ADDR_WIDTH-1:0] output_addr,
    input  logic [15:0]           out_channels,
    // Memory interface
    output logic                  dma_read_req,
    output logic [ADDR_WIDTH-1:0] dma_read_addr,
    input  logic [DATA_WIDTH-1:0] dma_read_data,
    input  logic                  dma_read_valid,
    output logic                  dma_write_req,
    output logic [ADDR_WIDTH-1:0] dma_write_addr,
    output logic [DATA_WIDTH-1:0] dma_write_data,
    input  logic                  dma_write_ack,
    input  logic                  vreg_ready,
    // Feedback from activation unit
    input  logic [DATA_WIDTH-1:0] act_data,
    input  logic                  act_valid,
    // Control handshake
    input  logic                  processing_done,
    output logic                  done
);

    typedef enum logic [2:0] {
        IDLE,
        LOAD_INPUT,
        LOAD_WEIGHTS,
        LOAD_BN_PARAMS,
        WAIT_PROCESS,
        STORE_OUTPUT
    } dma_state_t;
    
    dma_state_t current_state, next_state;
    
    // Address calculation constants
    localparam IMG_SIZE = IMG_WIDTH * IMG_HEIGHT;
    localparam WEIGHT_SIZE = 9; // 3x3 filter
    
    // Internal registers
    logic [31:0] transfer_count;
    logic [15:0] current_channel;
    logic [ADDR_WIDTH-1:0] current_addr;
    
    // Buffer for activation data to ensure it's properly captured
    logic [DATA_WIDTH-1:0] act_data_buffer;
    logic act_valid_buffer;
    logic output_writing_active;
    
    // Data buffering for activation output
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            act_data_buffer <= '0;
            act_valid_buffer <= 1'b0;
            output_writing_active <= 1'b0;
        end else begin
            // Set valid flag when we get new data from activation unit
            if (act_valid) begin
                act_data_buffer <= act_data;
                act_valid_buffer <= 1'b1;
            end else if (dma_write_ack && act_valid_buffer) begin
                // Clear valid flag after write is acknowledged
                act_valid_buffer <= 1'b0;
            end
            
            // Track if we're actively writing outputs
            if (current_state == STORE_OUTPUT) begin
                output_writing_active <= 1'b1;
            end else begin
                output_writing_active <= 1'b0;
            end
        end
    end
    
    // State register
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
            transfer_count <= '0;
            current_channel <= '0;
            current_addr <= '0;
            done <= 1'b0;
        end else begin
            current_state <= next_state;
            
            // Transfer counter logic
            if (next_state != current_state) begin
                transfer_count <= '0;
            end else if ((current_state == LOAD_INPUT || current_state == LOAD_WEIGHTS || 
                        current_state == LOAD_BN_PARAMS) && dma_read_valid) begin
                transfer_count <= transfer_count + 1;
            end else if (current_state == STORE_OUTPUT && dma_write_ack) begin
                transfer_count <= transfer_count + 1;
            end
            
            // Channel counter logic
            if (next_state == IDLE && current_state != IDLE) begin
                current_channel <= '0;
                done <= 1'b1;
            end else if (next_state == LOAD_INPUT && current_state == STORE_OUTPUT) begin
                current_channel <= current_channel + 1;
                done <= 1'b0;
            end else if (start && current_state == IDLE) begin
                done <= 1'b0;
            end
            
            // Address update
            if (next_state != current_state) begin
                case (next_state)
                    LOAD_INPUT:    current_addr <= input_addr + (IMG_SIZE * current_channel * DATA_WIDTH/8);
                    LOAD_WEIGHTS:  current_addr <= weight_addr + (WEIGHT_SIZE * current_channel * DATA_WIDTH/8);
                    LOAD_BN_PARAMS: current_addr <= bn_param_addr + (4 * current_channel * DATA_WIDTH/8); // 4 params per channel
                    STORE_OUTPUT:  current_addr <= output_addr + (IMG_SIZE * current_channel * DATA_WIDTH/8);
                    default:       current_addr <= current_addr;
                endcase
            end else if ((current_state == LOAD_INPUT || current_state == LOAD_WEIGHTS || 
                       current_state == LOAD_BN_PARAMS) && dma_read_valid) begin
                current_addr <= current_addr + (DATA_WIDTH/8);
            end else if (current_state == STORE_OUTPUT && dma_write_ack) begin
                current_addr <= current_addr + (DATA_WIDTH/8);
            end
        end
    end

    // Next state logic
    always_comb begin
        next_state = current_state;
        
        case (current_state)
            IDLE: begin
                if (start)
                    next_state = LOAD_INPUT;
            end
            
            LOAD_INPUT: begin
                if ((transfer_count >= IMG_SIZE - 1) && dma_read_valid)
                    next_state = LOAD_WEIGHTS;
            end
            
            LOAD_WEIGHTS: begin
                if ((transfer_count >= WEIGHT_SIZE - 1) && dma_read_valid)
                    next_state = LOAD_BN_PARAMS;
            end
            
            LOAD_BN_PARAMS: begin
                if ((transfer_count >= 3) && dma_read_valid)
                    next_state = WAIT_PROCESS;
            end
            
            WAIT_PROCESS: begin
                if (processing_done)
                    next_state = STORE_OUTPUT;
            end
            
            STORE_OUTPUT: begin
                if ((transfer_count >= IMG_SIZE - 1) && dma_write_ack) begin
                    if (current_channel >= out_channels - 1)
                        next_state = IDLE;
                    else
                        next_state = LOAD_INPUT;
                end
            end
            
            default: next_state = IDLE;
        endcase
    end

    // Output logic
    always_comb begin
        // Default values - explicitly assign to prevent X propagation
        dma_read_req = 1'b0;
        dma_read_addr = current_addr;
        dma_write_req = 1'b0;
        dma_write_addr = current_addr;
        dma_write_data = act_valid ? act_data : act_data_buffer; // Ensure valid data is prioritized
        
        case (current_state)
            LOAD_INPUT, LOAD_WEIGHTS, LOAD_BN_PARAMS: begin
                dma_read_req = 1'b1;
            end
            
            STORE_OUTPUT: begin
                // Only assert write request if we have valid data buffered
                // or activation unit has valid data
                if (act_valid_buffer || act_valid) begin
                    dma_write_req = 1'b1;
                    dma_write_data = act_valid ? act_data : act_data_buffer;
                end
            end
            
            WAIT_PROCESS: begin
                // If activation unit has valid data and we're waiting for processor
                // we should start writing it directly
                if (act_valid) begin
                    dma_write_req = 1'b1;
                    dma_write_addr = output_addr + (transfer_count * (DATA_WIDTH/8));
                    dma_write_data = act_data;
                end
            end
            
            default: begin
                // No memory operations in other states
                dma_read_req = 1'b0;
                dma_write_req = 1'b0;
            end
        endcase
    end

endmodule
