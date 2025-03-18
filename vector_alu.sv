module vector_alu #(
    parameter DATA_WIDTH = 16,
    parameter IMG_WIDTH = 224,
    parameter IMG_HEIGHT = 224
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
    input  logic                  out_ready,
    // Configuration
    input  logic                  enable_bn,
    input  logic [15:0]           current_channel
);

    // Internal registers for BN parameters
    logic [DATA_WIDTH-1:0] bn_mean;
    logic [DATA_WIDTH-1:0] bn_var;
    logic [DATA_WIDTH-1:0] bn_gamma;
    logic [DATA_WIDTH-1:0] bn_beta;
    
    // Internal register for 1x1 conv weights (3x3)
    logic [DATA_WIDTH-1:0] weights [8:0];
    
    // Internal state
    typedef enum logic [1:0] {
        IDLE,
        LOAD_PARAMS,
        PROCESS,
        FINISH
    } alu_state_t;
    
    alu_state_t current_state, next_state;
    logic [3:0] param_counter;
    logic [3:0] weight_counter;
    logic processing_valid;
    logic params_loaded;  // Flag to indicate parameters are loaded
    
    // Internal calculation signals
    logic [31:0] conv_result;
    logic [31:0] bn_result;
    
    // 1x1 Convolution calculation - using combinational logic
    always_comb begin
        // Default values to prevent X propagation
        conv_result = '0;
        bn_result = '0;
        
        if (in_valid && current_state == PROCESS) begin
            // Basic convolution - multiply input with center weight (1x1 conv)
            conv_result = $signed(in_data) * $signed(weights[4]);
            
            // Batch normalization
            if (enable_bn) begin
                // BN formula: gamma * (x - mean) / sqrt(var + eps) + beta
                // Simplified for fixed-point implementation
                bn_result = $signed(conv_result - bn_mean);
                // Division by sqrt(var + eps) would be implemented as a multiplication 
                // with precomputed 1/sqrt(var + eps) in fixed-point
                bn_result = $signed(bn_result * bn_gamma / 256); // Fixed-point scaling factor
                bn_result = $signed(bn_result + bn_beta);
            end else begin
                bn_result = conv_result;
            end
        end
    end
    
    // State register and main sequential logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
            param_counter <= '0;
            weight_counter <= '0;
            bn_mean <= '0;
            bn_var <= '0;
            bn_gamma <= '0;
            bn_beta <= '0;
            params_loaded <= 1'b0;
            out_data <= '0;
            out_valid <= 1'b0;
            processing_valid <= 1'b0;
            
            for (int i = 0; i < 9; i++) begin
                weights[i] <= '0;
            end
        end else begin
            current_state <= next_state;
            
            // Parameter and weight loading
            if (current_state == LOAD_PARAMS && in_valid) begin
                if (param_counter < 4) begin
                    case (param_counter)
                        0: bn_mean <= in_data;
                        1: bn_var <= in_data;
                        2: bn_gamma <= in_data;
                        3: bn_beta <= in_data;
                    endcase
                    param_counter <= param_counter + 1;
                end else if (weight_counter < 9) begin
                    weights[weight_counter] <= in_data;
                    weight_counter <= weight_counter + 1;
                end
            end
            
            // Update params_loaded flag
            if (param_counter >= 4 && weight_counter >= 4) begin
                params_loaded <= 1'b1;
            end
            
            // Reset counters and flags when returning to idle
            if (next_state == IDLE) begin
                param_counter <= '0;
                weight_counter <= '0;
                params_loaded <= 1'b0;
            end
            
            // Output handling
            processing_valid <= (current_state == PROCESS) && in_valid;
            out_valid <= processing_valid;
            
            // Register the output data calculated in the combinational block
            if (processing_valid) begin
                // Convert back to DATA_WIDTH (with saturation)
                if ($signed(bn_result) > $signed({1'b0, {(DATA_WIDTH-1){1'b1}}}))
                    out_data <= {1'b0, {(DATA_WIDTH-1){1'b1}}}; // Positive saturation
                else if ($signed(bn_result) < $signed({1'b1, {(DATA_WIDTH-1){1'b0}}}))
                    out_data <= {1'b1, {(DATA_WIDTH-1){1'b0}}}; // Negative saturation
                else
                    out_data <= bn_result[DATA_WIDTH-1:0];
            end
        end
    end
    
    // Next state logic
    always_comb begin
        next_state = current_state;
        
        case (current_state)
            IDLE: begin
                if (in_valid)
                    next_state = LOAD_PARAMS;
            end
            
            LOAD_PARAMS: begin
                // Use the flag instead of checking counters here
                if (params_loaded)
                    next_state = PROCESS;
            end
            
            PROCESS: begin
                if (!in_valid || !out_ready)
                    next_state = FINISH;
            end
            
            FINISH: begin
                if (!in_valid && !out_valid)
                    next_state = IDLE;
                else if (in_valid)
                    next_state = PROCESS;
            end
            
            default: next_state = IDLE;
        endcase
    end
    
    // Input ready logic
    assign in_ready = (current_state == IDLE) || (current_state == LOAD_PARAMS) || 
                    ((current_state == PROCESS) && out_ready);
    
endmodule
