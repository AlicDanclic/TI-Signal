// GF(2^4)域乘法核心模块
// 用于AES S-box复合域实现中的GF(2^4)乘法运算

module mul_core(
        input [3:0] d1,
        input [3:0] d2,
        output wire[3:0] mul_core);
    wire a,b;

    assign	a=d1[0]^d1[3];
    assign	b=d1[2]^d1[3];

    assign	mul_core[0]=(d1[0]&d2[0])^(d1[3]&d2[1])^(d1[2]&d2[2])^(d1[1]&d2[3]);
    assign	mul_core[1]=(d1[1]&d2[0])^(a&d2[1])^(b&d2[2])^((d1[1]^d1[2])&d2[3]);
    assign	mul_core[2]=(d1[2]&d2[0])^(d1[1]&d2[1])^(a&d2[2])^(b&d2[3]);
    assign	mul_core[3]=(d1[3]&d2[0])^(d1[2]&d2[1])^(d1[1]&d2[2])^(a&d2[3]);

endmodule
