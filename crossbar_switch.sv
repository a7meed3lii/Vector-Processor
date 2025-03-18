module crossbar_switch #(
    parameter DATA_WIDTH = 16,
    parameter NUM_INPUTS = 8,
    parameter NUM_OUTPUTS = 8
)(
    input  logic                          clk,
    input  logic                          rst_n,
    // Input interface
    input  logic [DATA_WIDTH-1:0]         in_data,
    input  logic                          in_valid,
    output logic                          in_ready,
    // Feedback input interface
    input  logic [DATA_WIDTH-1:0]         feedback_data,
    input  logic                          feedback_valid,
    // Output interface
    output logic [DATA_WIDTH-1:0]         out_data,
    output logic                          out_valid,
    input  logic                          out_ready,
    // Configuration
    input  logic [1:0]                    config_sel
);

    // Internal registers for routing configuration
    logic [NUM_OUTPUTS-1:0][NUM_INPUTS-1:0] routing_matrix;
    logic [$clog2(NUM_INPUTS)-1:0] src_select [NUM_OUTPUTS-1:0];
    
    // Routing matrix based on configuration
    always_comb begin
        // Default configuration: direct connection
        for (int i = 0; i < NUM_OUTPUTS; i++) begin
            for (int j = 0; j < NUM_INPUTS; j++) begin
                routing_matrix[i][j] = (i == j) ? 1'b1 : 1'b0;
            end
        end
        
        // Modify routing based on configuration select
        case (config_sel)
            2'b00: begin
                // Default configuration (identity)
            end
            
            2'b01: begin
                // Configuration for loading data
                for (int i = 0; i < NUM_OUTPUTS; i++) begin
                    for (int j = 0; j < NUM_INPUTS; j++) begin
                        routing_matrix[i][j] = (j == 0) ? 1'b1 : 1'b0; // All outputs connect to first input
                    end
                end
            end
            
            2'b10: begin
                // Configuration for processing
                // Example: Shift matrix - each output connects to input i+1 mod NUM_INPUTS
                for (int i = 0; i < NUM_OUTPUTS; i++) begin
                    for (int j = 0; j < NUM_INPUTS; j++) begin
                        routing_matrix[i][j] = (j == ((i+1) % NUM_INPUTS)) ? 1'b1 : 1'b0;
                    end
                end
            end
            
            2'b11: begin
                // Configuration for feedback
                // Example: Broadcast - all outputs connect to last input
                for (int i = 0; i < NUM_OUTPUTS; i++) begin
                    for (int j = 0; j < NUM_INPUTS; j++) begin
                        routing_matrix[i][j] = (j == NUM_INPUTS-1) ? 1'b1 : 1'b0;
                    end
                end
            end
        endcase
    end
    
    // Convert routing matrix to selector signals
    always_comb begin
        for (int i = 0; i < NUM_OUTPUTS; i++) begin
            src_select[i] = '0;
            for (int j = 0; j < NUM_INPUTS; j++) begin
                if (routing_matrix[i][j]) begin
                    src_select[i] = j[$clog2(NUM_INPUTS)-1:0];
                    break;
                end
            end
        end
    end
    
    // Implement the actual crossbar switching functionality
    always_comb begin
        if (config_sel == 2'b11 && feedback_valid) begin
            // Use feedback data when in feedback mode and feedback is valid
            out_data = feedback_data;
            out_valid = feedback_valid && out_ready;
        end else begin
            // Normal operation (use input data)
            out_data = in_data;
            out_valid = in_valid && out_ready;
        end
    end
    
    // Ready when downstream is ready
    assign in_ready = out_ready;

endmodule
