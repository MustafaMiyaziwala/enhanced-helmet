module root(input sys_clk, input sys_rst_n, output trig_0, output echo_0);

    reg [4:0] ultra_clk;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            ultra_clk <= 5'd0;
        else if (counter < 5'd24) // 1 us clock period
            ultra_clk <= ultra_clk + 1;
        else
            ultra_clk <= 5'd0;
    end


    reg [5:0] pulse_0;

    ultrasonic ultra_0(pulse_0, trig_0, echo_0, sys_rst_n, ultra_clk);


endmodule