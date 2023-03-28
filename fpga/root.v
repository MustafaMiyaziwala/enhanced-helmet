module root(input sys_clk, input sys_rst_n, output trig_0, input echo_0, output test_0);

    reg [3:0] counter;
    reg ultra_clk;

    initial begin
        ultra_clk <= 0;
    end

    always @(posedge sys_clk ) begin
        if (counter < 5'd11) // 1 us clock period
            counter <= counter + 1;
        else begin
            counter <= 5'd0;
            ultra_clk <= ~ultra_clk;
        end
    end


    ultrasonic ultra_0(trig_0, echo_0, sys_rst_n, ultra_clk, test_0);

    //assign trig_0 = echo_0;


endmodule  
