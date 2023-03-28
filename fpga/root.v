module root(input sys_clk, input sys_rst_n, output trig_0, input echo_0, output test_0);

    reg [3:0] ultra_prescaler;
    reg ultra_clk;

    initial begin
        ultra_clk <= 0;
    end

    always @(posedge sys_clk ) begin
        if (ultra_prescaler < 5'd11) // 1 us clock period
            ultra_prescaler <= ultra_prescaler + 1;
        else begin
            ultra_prescaler <= 5'd0;
            ultra_clk <= ~ultra_clk;
        end
    end

    wire [14:0] pulse;

    ultrasonic ultra_0(ultra_clk, echo_0, trigger_0, pulse);

    assign test_0 = pulse < 50;

endmodule  
