module activation_unit #(
    parameter DATA_WIDTH = 16
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
    input  logic                  use_relu  // 0: H-swish, 1: ReLU
);

    // Temporary calculation signals
    logic [DATA_WIDTH-1:0] result_relu;
    logic [DATA_WIDTH-1:0] result_hswish;
    logic [DATA_WIDTH-1:0] x_plus_3;
    logic [DATA_WIDTH-1:0] relu6_result;
    logic [31:0] mult_result;
    
    // Combinational calculation logic to prevent X propagation
    always_comb begin
        // Initialize values to prevent X propagation
        result_relu = '0;
        result_hswish = '0;
        x_plus_3 = '0;
        relu6_result = '0;
        mult_result = '0;
        
        // Calculate ReLU
        result_relu = (in_data[DATA_WIDTH-1]) ? '0 : in_data; // If MSB (sign bit) is 1, return 0, else return x
        
        // Calculate H-swish
        // For fixed-point implementation, we'll use a simplified approach
        x_plus_3 = in_data + 16'h0300; // 3.0 in fixed-point (8 fraction bits)
        
        // ReLU6
        if (x_plus_3[DATA_WIDTH-1]) begin // Negative
            relu6_result = '0;
        end else if ($signed(x_plus_3) > $signed(16'h0600)) begin // Greater than 6.0
            relu6_result = 16'h0600; // 6.0 in fixed-point
        end else begin
            relu6_result = x_plus_3;
        end
        
        // Multiply and divide by 6
        mult_result = $signed(in_data) * $signed(relu6_result);
        
        // Return with proper scaling (divide by 6 and handle fixed-point scaling)
        result_hswish = mult_result[DATA_WIDTH+8-1:8] / 6;
    end
    
    // Sequential logic for output generation
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_data <= '0;
            out_valid <= 1'b0;
        end else begin
            if (in_valid && in_ready) begin
                // Select activation function based on configuration
                if (use_relu)
                    out_data <= result_relu;
                else
                    out_data <= result_hswish;
                    
                out_valid <= 1'b1;
            end else if (out_valid && out_ready) begin
                out_valid <= 1'b0;
            end
        end
    end
    
    // Input ready when we can accept data
    assign in_ready = !out_valid || out_ready;

endmodule
