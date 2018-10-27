-- This file was generated with hex2rom written by Daniel Wallner

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity bootrom is
	port(
		A	: in std_logic_vector(9 downto 0);
		D	: out std_logic_vector(31 downto 0)
	);
end bootrom;

architecture rtl of bootrom is
begin
	process (A)
	begin
		case to_integer(unsigned(A)) is
		when 000000 => D <= "00000001010000000000000001101111";	-- 0x0000
		when 000001 => D <= "00000000000000000000000001101111";	-- 0x0004
		when 000002 => D <= "00000000000000000000000000010011";	-- 0x0008
		when 000003 => D <= "00000000000000000000000000010011";	-- 0x000C
		when 000004 => D <= "00000000000000000000000001101111";	-- 0x0010
		when 000005 => D <= "00000000000000000000000110010011";	-- 0x0014
		when 000006 => D <= "00000000000000000000001010010011";	-- 0x0018
		when 000007 => D <= "11111110010100000001010011100011";	-- 0x001C
		when 000008 => D <= "00000000000000000000000010010111";	-- 0x0020
		when 000009 => D <= "01111110000000001000000010010011";	-- 0x0024
		when 000010 => D <= "00000000000000000001000100010111";	-- 0x0028
		when 000011 => D <= "10000001100000010000000100010011";	-- 0x002C
		when 000012 => D <= "00000000000000001010000110000011";	-- 0x0030
		when 000013 => D <= "00000000000000000000001000010011";	-- 0x0034
		when 000014 => D <= "00000000000100000000001010010011";	-- 0x0038
		when 000015 => D <= "11111111111100000000001100010011";	-- 0x003C
		when 000016 => D <= "10000000000000000000001110110111";	-- 0x0040
		when 000017 => D <= "11111111111100111000001110010011";	-- 0x0044
		when 000018 => D <= "10000000000000000000010000110111";	-- 0x0048
		when 000019 => D <= "00000000010000011000001000110011";	-- 0x004C
		when 000020 => D <= "00000000010100011000001010110011";	-- 0x0050
		when 000021 => D <= "00000000011000011000001100110011";	-- 0x0054
		when 000022 => D <= "00000000011100011000001110110011";	-- 0x0058
		when 000023 => D <= "00000000100000011000010000110011";	-- 0x005C
		when 000024 => D <= "00000000001100010010000000100011";	-- 0x0060
		when 000025 => D <= "00000000010000010010001000100011";	-- 0x0064
		when 000026 => D <= "00000000010100010010010000100011";	-- 0x0068
		when 000027 => D <= "00000000011000010010011000100011";	-- 0x006C
		when 000028 => D <= "00000000011100010010100000100011";	-- 0x0070
		when 000029 => D <= "00000000100000010010101000100011";	-- 0x0074
		when 000030 => D <= "00000000000000000000001010010011";	-- 0x0078
		when 000031 => D <= "11111000010100011001010011100011";	-- 0x007C
		when 000032 => D <= "00000000000000000000001010010011";	-- 0x0080
		when 000033 => D <= "11111000010100100001000011100011";	-- 0x0084
		when 000034 => D <= "00000000000000000000001010010011";	-- 0x0088
		when 000035 => D <= "11110110010100101001110011100011";	-- 0x008C
		when 000036 => D <= "11111111111100000000001010010011";	-- 0x0090
		when 000037 => D <= "11110110010100110001100011100011";	-- 0x0094
		when 000038 => D <= "10000000000000000000001010110111";	-- 0x0098
		when 000039 => D <= "11111111111100101000001010010011";	-- 0x009C
		when 000040 => D <= "11110110010100111001001011100011";	-- 0x00A0
		when 000041 => D <= "10000000000000000000001010110111";	-- 0x00A4
		when 000042 => D <= "11110100010101000001111011100011";	-- 0x00A8
		when 000043 => D <= "00000000000000000000000010010111";	-- 0x00AC
		when 000044 => D <= "01110101100000001000000010010011";	-- 0x00B0
		when 000045 => D <= "00000000000000000000000100010111";	-- 0x00B4
		when 000046 => D <= "01111010010000010000000100010011";	-- 0x00B8
		when 000047 => D <= "00000000000000001010010000000011";	-- 0x00BC
		when 000048 => D <= "00000000000000000000010010010011";	-- 0x00C0
		when 000049 => D <= "00000000000100000000010100010011";	-- 0x00C4
		when 000050 => D <= "11111111111100000000010110010011";	-- 0x00C8
		when 000051 => D <= "10000000000000000000011000110111";	-- 0x00CC
		when 000052 => D <= "11111111111101100000011000010011";	-- 0x00D0
		when 000053 => D <= "10000000000000000000011010110111";	-- 0x00D4
		when 000054 => D <= "00000000100101000000010010110011";	-- 0x00D8
		when 000055 => D <= "00000000101001000000010100110011";	-- 0x00DC
		when 000056 => D <= "00000000101101000000010110110011";	-- 0x00E0
		when 000057 => D <= "00000000110001000000011000110011";	-- 0x00E4
		when 000058 => D <= "00000000110101000000011010110011";	-- 0x00E8
		when 000059 => D <= "00000000100000010010000000100011";	-- 0x00EC
		when 000060 => D <= "00000000100100010010001000100011";	-- 0x00F0
		when 000061 => D <= "00000000101000010010010000100011";	-- 0x00F4
		when 000062 => D <= "00000000101100010010011000100011";	-- 0x00F8
		when 000063 => D <= "00000000110000010010100000100011";	-- 0x00FC
		when 000064 => D <= "00000000110100010010101000100011";	-- 0x0100
		when 000065 => D <= "00000000000100000000001010010011";	-- 0x0104
		when 000066 => D <= "11101110010101000001111011100011";	-- 0x0108
		when 000067 => D <= "00000000000100000000001010010011";	-- 0x010C
		when 000068 => D <= "11101110010101001001101011100011";	-- 0x0110
		when 000069 => D <= "00000000000000000000001010010011";	-- 0x0114
		when 000070 => D <= "11101110010101011001011011100011";	-- 0x0118
		when 000071 => D <= "10000000000000000000001010110111";	-- 0x011C
		when 000072 => D <= "11101110010101100001001011100011";	-- 0x0120
		when 000073 => D <= "10000000000000000000001010110111";	-- 0x0124
		when 000074 => D <= "00000000000100101000001010010011";	-- 0x0128
		when 000075 => D <= "11101100010101101001110011100011";	-- 0x012C
		when 000076 => D <= "00000000000000000000000010010111";	-- 0x0130
		when 000077 => D <= "01101101100000001000000010010011";	-- 0x0134
		when 000078 => D <= "00000000000000000000000100010111";	-- 0x0138
		when 000079 => D <= "01110011100000010000000100010011";	-- 0x013C
		when 000080 => D <= "00000000000000001010011010000011";	-- 0x0140
		when 000081 => D <= "00000000000000000000011100010011";	-- 0x0144
		when 000082 => D <= "00000000000100000000011110010011";	-- 0x0148
		when 000083 => D <= "11111111111100000000100000010011";	-- 0x014C
		when 000084 => D <= "10000000000000000000100010110111";	-- 0x0150
		when 000085 => D <= "11111111111110001000100010010011";	-- 0x0154
		when 000086 => D <= "10000000000000000000100100110111";	-- 0x0158
		when 000087 => D <= "00000000111001101000011100110011";	-- 0x015C
		when 000088 => D <= "00000000111101101000011110110011";	-- 0x0160
		when 000089 => D <= "00000001000001101000100000110011";	-- 0x0164
		when 000090 => D <= "00000001000101101000100010110011";	-- 0x0168
		when 000091 => D <= "00000001001001101000100100110011";	-- 0x016C
		when 000092 => D <= "00000000110100010010000000100011";	-- 0x0170
		when 000093 => D <= "00000000111000010010001000100011";	-- 0x0174
		when 000094 => D <= "00000000111100010010010000100011";	-- 0x0178
		when 000095 => D <= "00000001000000010010011000100011";	-- 0x017C
		when 000096 => D <= "00000001000100010010100000100011";	-- 0x0180
		when 000097 => D <= "00000001001000010010101000100011";	-- 0x0184
		when 000098 => D <= "11111111111100000000001010010011";	-- 0x0188
		when 000099 => D <= "11100110010101101001110011100011";	-- 0x018C
		when 000100 => D <= "11111111111100000000001010010011";	-- 0x0190
		when 000101 => D <= "11100110010101110001100011100011";	-- 0x0194
		when 000102 => D <= "00000000000000000000001010010011";	-- 0x0198
		when 000103 => D <= "11100110010101111001010011100011";	-- 0x019C
		when 000104 => D <= "11111111111000000000001010010011";	-- 0x01A0
		when 000105 => D <= "11100110010110000001000011100011";	-- 0x01A4
		when 000106 => D <= "10000000000000000000001010110111";	-- 0x01A8
		when 000107 => D <= "11111111111000101000001010010011";	-- 0x01AC
		when 000108 => D <= "11100100010110001001101011100011";	-- 0x01B0
		when 000109 => D <= "10000000000000000000001010110111";	-- 0x01B4
		when 000110 => D <= "11111111111100101000001010010011";	-- 0x01B8
		when 000111 => D <= "11100100010110010001010011100011";	-- 0x01BC
		when 000112 => D <= "00000000000000000000000010010111";	-- 0x01C0
		when 000113 => D <= "01100100110000001000000010010011";	-- 0x01C4
		when 000114 => D <= "00000000000000000000000100010111";	-- 0x01C8
		when 000115 => D <= "01101100000000010000000100010011";	-- 0x01CC
		when 000116 => D <= "00000000000000001010100100000011";	-- 0x01D0
		when 000117 => D <= "00000000000000000000100110010011";	-- 0x01D4
		when 000118 => D <= "00000000000100000000101000010011";	-- 0x01D8
		when 000119 => D <= "11111111111100000000101010010011";	-- 0x01DC
		when 000120 => D <= "10000000000000000000101100110111";	-- 0x01E0
		when 000121 => D <= "11111111111110110000101100010011";	-- 0x01E4
		when 000122 => D <= "10000000000000000000101110110111";	-- 0x01E8
		when 000123 => D <= "00000001001110010000100110110011";	-- 0x01EC
		when 000124 => D <= "00000001010010010000101000110011";	-- 0x01F0
		when 000125 => D <= "00000001010110010000101010110011";	-- 0x01F4
		when 000126 => D <= "00000001011010010000101100110011";	-- 0x01F8
		when 000127 => D <= "00000001011110010000101110110011";	-- 0x01FC
		when 000128 => D <= "00000001001000010010000000100011";	-- 0x0200
		when 000129 => D <= "00000001001100010010001000100011";	-- 0x0204
		when 000130 => D <= "00000001010000010010010000100011";	-- 0x0208
		when 000131 => D <= "00000001010100010010011000100011";	-- 0x020C
		when 000132 => D <= "00000001011000010010100000100011";	-- 0x0210
		when 000133 => D <= "00000001011100010010101000100011";	-- 0x0214
		when 000134 => D <= "10000000000000000000001010110111";	-- 0x0218
		when 000135 => D <= "11111111111100101000001010010011";	-- 0x021C
		when 000136 => D <= "11011110010110010001001011100011";	-- 0x0220
		when 000137 => D <= "10000000000000000000001010110111";	-- 0x0224
		when 000138 => D <= "11111111111100101000001010010011";	-- 0x0228
		when 000139 => D <= "11011100010110011001110011100011";	-- 0x022C
		when 000140 => D <= "10000000000000000000001010110111";	-- 0x0230
		when 000141 => D <= "11011100010110100001100011100011";	-- 0x0234
		when 000142 => D <= "10000000000000000000001010110111";	-- 0x0238
		when 000143 => D <= "11111111111000101000001010010011";	-- 0x023C
		when 000144 => D <= "11011100010110101001001011100011";	-- 0x0240
		when 000145 => D <= "11111111111000000000001010010011";	-- 0x0244
		when 000146 => D <= "11011010010110110001111011100011";	-- 0x0248
		when 000147 => D <= "11111111111100000000001010010011";	-- 0x024C
		when 000148 => D <= "11011010010110111001101011100011";	-- 0x0250
		when 000149 => D <= "00000000000000000000000010010111";	-- 0x0254
		when 000150 => D <= "01011011110000001000000010010011";	-- 0x0258
		when 000151 => D <= "00000000000000000000000100010111";	-- 0x025C
		when 000152 => D <= "01100100010000010000000100010011";	-- 0x0260
		when 000153 => D <= "00000000000000001010101110000011";	-- 0x0264
		when 000154 => D <= "00000000000000000000110000010011";	-- 0x0268
		when 000155 => D <= "00000000000100000000110010010011";	-- 0x026C
		when 000156 => D <= "11111111111100000000110100010011";	-- 0x0270
		when 000157 => D <= "10000000000000000000110110110111";	-- 0x0274
		when 000158 => D <= "11111111111111011000110110010011";	-- 0x0278
		when 000159 => D <= "10000000000000000000111000110111";	-- 0x027C
		when 000160 => D <= "00000001100010111000110000110011";	-- 0x0280
		when 000161 => D <= "00000001100110111000110010110011";	-- 0x0284
		when 000162 => D <= "00000001101010111000110100110011";	-- 0x0288
		when 000163 => D <= "00000001101110111000110110110011";	-- 0x028C
		when 000164 => D <= "00000001110010111000111000110011";	-- 0x0290
		when 000165 => D <= "00000001011100010010000000100011";	-- 0x0294
		when 000166 => D <= "00000001100000010010001000100011";	-- 0x0298
		when 000167 => D <= "00000001100100010010010000100011";	-- 0x029C
		when 000168 => D <= "00000001101000010010011000100011";	-- 0x02A0
		when 000169 => D <= "00000001101100010010100000100011";	-- 0x02A4
		when 000170 => D <= "00000001110000010010101000100011";	-- 0x02A8
		when 000171 => D <= "10000000000000000000001010110111";	-- 0x02AC
		when 000172 => D <= "11010100010110111001101011100011";	-- 0x02B0
		when 000173 => D <= "10000000000000000000001010110111";	-- 0x02B4
		when 000174 => D <= "11010100010111000001011011100011";	-- 0x02B8
		when 000175 => D <= "10000000000000000000001010110111";	-- 0x02BC
		when 000176 => D <= "00000000000100101000001010010011";	-- 0x02C0
		when 000177 => D <= "11010100010111001001000011100011";	-- 0x02C4
		when 000178 => D <= "10000000000000000000001010110111";	-- 0x02C8
		when 000179 => D <= "11111111111100101000001010010011";	-- 0x02CC
		when 000180 => D <= "11010010010111010001101011100011";	-- 0x02D0
		when 000181 => D <= "11111111111100000000001010010011";	-- 0x02D4
		when 000182 => D <= "11010010010111011001011011100011";	-- 0x02D8
		when 000183 => D <= "00000000000000000000001010010011";	-- 0x02DC
		when 000184 => D <= "11010010010111100001001011100011";	-- 0x02E0
		when 000185 => D <= "00000000000000000000110010010111";	-- 0x02E4
		when 000186 => D <= "01010011000011001000110010010011";	-- 0x02E8
		when 000187 => D <= "00000000000000000000110100010111";	-- 0x02EC
		when 000188 => D <= "01011100110011010000110100010011";	-- 0x02F0
		when 000189 => D <= "00000000000011001010111000000011";	-- 0x02F4
		when 000190 => D <= "00000000000100000000110110010011";	-- 0x02F8
		when 000191 => D <= "00000001101111100000111010110011";	-- 0x02FC
		when 000192 => D <= "00000001101111101000111100110011";	-- 0x0300
		when 000193 => D <= "00000001101111110000111110110011";	-- 0x0304
		when 000194 => D <= "00000001101111111000000010110011";	-- 0x0308
		when 000195 => D <= "00000001101100001000000100110011";	-- 0x030C
		when 000196 => D <= "00000001101100010000000110110011";	-- 0x0310
		when 000197 => D <= "00000001101111010010000000100011";	-- 0x0314
		when 000198 => D <= "00000001110011010010001000100011";	-- 0x0318
		when 000199 => D <= "00000001110111010010010000100011";	-- 0x031C
		when 000200 => D <= "00000001111011010010011000100011";	-- 0x0320
		when 000201 => D <= "00000001111111010010100000100011";	-- 0x0324
		when 000202 => D <= "00000000000111010010101000100011";	-- 0x0328
		when 000203 => D <= "00000000001011010010110000100011";	-- 0x032C
		when 000204 => D <= "00000000001111010010111000100011";	-- 0x0330
		when 000205 => D <= "00000000000100000000001010010011";	-- 0x0334
		when 000206 => D <= "11001100010111011001011011100011";	-- 0x0338
		when 000207 => D <= "00000000000000001011001010110111";	-- 0x033C
		when 000208 => D <= "10111100110100101000001010010011";	-- 0x0340
		when 000209 => D <= "11001100010111100001000011100011";	-- 0x0344
		when 000210 => D <= "00000000000000001011001010110111";	-- 0x0348
		when 000211 => D <= "10111100111000101000001010010011";	-- 0x034C
		when 000212 => D <= "11001010010111101001101011100011";	-- 0x0350
		when 000213 => D <= "00000000000000001011001010110111";	-- 0x0354
		when 000214 => D <= "10111100111100101000001010010011";	-- 0x0358
		when 000215 => D <= "11001010010111110001010011100011";	-- 0x035C
		when 000216 => D <= "00000000000000001011001010110111";	-- 0x0360
		when 000217 => D <= "10111101000000101000001010010011";	-- 0x0364
		when 000218 => D <= "11001000010111111001111011100011";	-- 0x0368
		when 000219 => D <= "00000000000000001011001010110111";	-- 0x036C
		when 000220 => D <= "10111101001000101000001010010011";	-- 0x0370
		when 000221 => D <= "11001000010100010001100011100011";	-- 0x0374
		when 000222 => D <= "00000000000000001011001010110111";	-- 0x0378
		when 000223 => D <= "10111101001100101000001010010011";	-- 0x037C
		when 000224 => D <= "11001000010100011001001011100011";	-- 0x0380
		when 000225 => D <= "00000000000000000000000010010111";	-- 0x0384
		when 000226 => D <= "01001001010000001000000010010011";	-- 0x0388
		when 000227 => D <= "00000000000000000000000100010111";	-- 0x038C
		when 000228 => D <= "01010100110000010000000100010011";	-- 0x0390
		when 000229 => D <= "00000000000000001010111000000011";	-- 0x0394
		when 000230 => D <= "11110111111111111001110110110111";	-- 0x0398
		when 000231 => D <= "10000001100011011000110110010011";	-- 0x039C
		when 000232 => D <= "00000001101111100000000000110011";	-- 0x03A0
		when 000233 => D <= "00000000000000010010000000100011";	-- 0x03A4
		when 000234 => D <= "00000000000000000000001010010011";	-- 0x03A8
		when 000235 => D <= "11000100010100000001110011100011";	-- 0x03AC
		when 000236 => D <= "00000000000000000000000010010111";	-- 0x03B0
		when 000237 => D <= "01000110110000001000000010010011";	-- 0x03B4
		when 000238 => D <= "00000000000000000000000100010111";	-- 0x03B8
		when 000239 => D <= "01010010010000010000000100010011";	-- 0x03BC
		when 000240 => D <= "00000000000000001010111000000011";	-- 0x03C0
		when 000241 => D <= "11110111111111111001110110110111";	-- 0x03C4
		when 000242 => D <= "10000001100011011000110110010011";	-- 0x03C8
		when 000243 => D <= "00000001101111100000000000110011";	-- 0x03CC
		when 000244 => D <= "00000000000000000000001010110011";	-- 0x03D0
		when 000245 => D <= "00000000000000010010000000100011";	-- 0x03D4
		when 000246 => D <= "00000000010100010010001000100011";	-- 0x03D8
		when 000247 => D <= "00000000000000000000001010010011";	-- 0x03DC
		when 000248 => D <= "11000010010100000001001011100011";	-- 0x03E0
		when 000249 => D <= "00000000000000000000001010010011";	-- 0x03E4
		when 000250 => D <= "11000000010100101001111011100011";	-- 0x03E8
		when 000251 => D <= "00000000000000000000000010010111";	-- 0x03EC
		when 000252 => D <= "01000011010000001000000010010011";	-- 0x03F0
		when 000253 => D <= "00000000000000000000000100010111";	-- 0x03F4
		when 000254 => D <= "01001111000000010000000100010011";	-- 0x03F8
		when 000255 => D <= "00000000000000001010000110000011";	-- 0x03FC
		when 000256 => D <= "00000000000000011000001000110011";	-- 0x0400
		when 000257 => D <= "00000000000000100000001010110011";	-- 0x0404
		when 000258 => D <= "00000000010100000000001100110011";	-- 0x0408
		when 000259 => D <= "00000000000000110000011100110011";	-- 0x040C
		when 000260 => D <= "00000000000001110000011110110011";	-- 0x0410
		when 000261 => D <= "00000000000001111000100000110011";	-- 0x0414
		when 000262 => D <= "00000001000000000000110010110011";	-- 0x0418
		when 000263 => D <= "00000001100100000000110100110011";	-- 0x041C
		when 000264 => D <= "00000000000011010000110110110011";	-- 0x0420
		when 000265 => D <= "00000000010000010010000000100011";	-- 0x0424
		when 000266 => D <= "00000001101000010010001000100011";	-- 0x0428
		when 000267 => D <= "00000001101100010010010000100011";	-- 0x042C
		when 000268 => D <= "00110110100100100110001010110111";	-- 0x0430
		when 000269 => D <= "10000001010000101000001010010011";	-- 0x0434
		when 000270 => D <= "10111100010100100001011011100011";	-- 0x0438
		when 000271 => D <= "00110110100100100110001010110111";	-- 0x043C
		when 000272 => D <= "10000001010000101000001010010011";	-- 0x0440
		when 000273 => D <= "10111100010111010001000011100011";	-- 0x0444
		when 000274 => D <= "00110110100100100110001010110111";	-- 0x0448
		when 000275 => D <= "10000001010000101000001010010011";	-- 0x044C
		when 000276 => D <= "10111010010111011001101011100011";	-- 0x0450
		when 000277 => D <= "10111011110111111111000001101111";	-- 0x0454
		when 000278 => D <= "11000000000000000001000001110011";	-- 0x0458
		when 000279 => D <= "00000000000000000000000000000000";	-- 0x045C
		when 000280 => D <= "00000000000000000000000000000000";	-- 0x0460
		when 000281 => D <= "00000000000000000000000000000000";	-- 0x0464
		when 000282 => D <= "00000000000000000000000000000000";	-- 0x0468
		when 000283 => D <= "00000000000000000000000000000000";	-- 0x046C
		when 000284 => D <= "00000000000000000000000000000000";	-- 0x0470
		when 000285 => D <= "00000000000000000000000000000000";	-- 0x0474
		when 000286 => D <= "00000000000000000000000000000000";	-- 0x0478
		when 000287 => D <= "00000000000000000000000000000000";	-- 0x047C
		when 000288 => D <= "--------------------------------";	-- 0x0480
		when 000289 => D <= "--------------------------------";	-- 0x0484
		when 000290 => D <= "--------------------------------";	-- 0x0488
		when 000291 => D <= "--------------------------------";	-- 0x048C
		when 000292 => D <= "--------------------------------";	-- 0x0490
		when 000293 => D <= "--------------------------------";	-- 0x0494
		when 000294 => D <= "--------------------------------";	-- 0x0498
		when 000295 => D <= "--------------------------------";	-- 0x049C
		when 000296 => D <= "--------------------------------";	-- 0x04A0
		when 000297 => D <= "--------------------------------";	-- 0x04A4
		when 000298 => D <= "--------------------------------";	-- 0x04A8
		when 000299 => D <= "--------------------------------";	-- 0x04AC
		when 000300 => D <= "--------------------------------";	-- 0x04B0
		when 000301 => D <= "--------------------------------";	-- 0x04B4
		when 000302 => D <= "--------------------------------";	-- 0x04B8
		when 000303 => D <= "--------------------------------";	-- 0x04BC
		when 000304 => D <= "--------------------------------";	-- 0x04C0
		when 000305 => D <= "--------------------------------";	-- 0x04C4
		when 000306 => D <= "--------------------------------";	-- 0x04C8
		when 000307 => D <= "--------------------------------";	-- 0x04CC
		when 000308 => D <= "--------------------------------";	-- 0x04D0
		when 000309 => D <= "--------------------------------";	-- 0x04D4
		when 000310 => D <= "--------------------------------";	-- 0x04D8
		when 000311 => D <= "--------------------------------";	-- 0x04DC
		when 000312 => D <= "--------------------------------";	-- 0x04E0
		when 000313 => D <= "--------------------------------";	-- 0x04E4
		when 000314 => D <= "--------------------------------";	-- 0x04E8
		when 000315 => D <= "--------------------------------";	-- 0x04EC
		when 000316 => D <= "--------------------------------";	-- 0x04F0
		when 000317 => D <= "--------------------------------";	-- 0x04F4
		when 000318 => D <= "--------------------------------";	-- 0x04F8
		when 000319 => D <= "--------------------------------";	-- 0x04FC
		when 000320 => D <= "--------------------------------";	-- 0x0500
		when 000321 => D <= "--------------------------------";	-- 0x0504
		when 000322 => D <= "--------------------------------";	-- 0x0508
		when 000323 => D <= "--------------------------------";	-- 0x050C
		when 000324 => D <= "--------------------------------";	-- 0x0510
		when 000325 => D <= "--------------------------------";	-- 0x0514
		when 000326 => D <= "--------------------------------";	-- 0x0518
		when 000327 => D <= "--------------------------------";	-- 0x051C
		when 000328 => D <= "--------------------------------";	-- 0x0520
		when 000329 => D <= "--------------------------------";	-- 0x0524
		when 000330 => D <= "--------------------------------";	-- 0x0528
		when 000331 => D <= "--------------------------------";	-- 0x052C
		when 000332 => D <= "--------------------------------";	-- 0x0530
		when 000333 => D <= "--------------------------------";	-- 0x0534
		when 000334 => D <= "--------------------------------";	-- 0x0538
		when 000335 => D <= "--------------------------------";	-- 0x053C
		when 000336 => D <= "--------------------------------";	-- 0x0540
		when 000337 => D <= "--------------------------------";	-- 0x0544
		when 000338 => D <= "--------------------------------";	-- 0x0548
		when 000339 => D <= "--------------------------------";	-- 0x054C
		when 000340 => D <= "--------------------------------";	-- 0x0550
		when 000341 => D <= "--------------------------------";	-- 0x0554
		when 000342 => D <= "--------------------------------";	-- 0x0558
		when 000343 => D <= "--------------------------------";	-- 0x055C
		when 000344 => D <= "--------------------------------";	-- 0x0560
		when 000345 => D <= "--------------------------------";	-- 0x0564
		when 000346 => D <= "--------------------------------";	-- 0x0568
		when 000347 => D <= "--------------------------------";	-- 0x056C
		when 000348 => D <= "--------------------------------";	-- 0x0570
		when 000349 => D <= "--------------------------------";	-- 0x0574
		when 000350 => D <= "--------------------------------";	-- 0x0578
		when 000351 => D <= "--------------------------------";	-- 0x057C
		when 000352 => D <= "--------------------------------";	-- 0x0580
		when 000353 => D <= "--------------------------------";	-- 0x0584
		when 000354 => D <= "--------------------------------";	-- 0x0588
		when 000355 => D <= "--------------------------------";	-- 0x058C
		when 000356 => D <= "--------------------------------";	-- 0x0590
		when 000357 => D <= "--------------------------------";	-- 0x0594
		when 000358 => D <= "--------------------------------";	-- 0x0598
		when 000359 => D <= "--------------------------------";	-- 0x059C
		when 000360 => D <= "--------------------------------";	-- 0x05A0
		when 000361 => D <= "--------------------------------";	-- 0x05A4
		when 000362 => D <= "--------------------------------";	-- 0x05A8
		when 000363 => D <= "--------------------------------";	-- 0x05AC
		when 000364 => D <= "--------------------------------";	-- 0x05B0
		when 000365 => D <= "--------------------------------";	-- 0x05B4
		when 000366 => D <= "--------------------------------";	-- 0x05B8
		when 000367 => D <= "--------------------------------";	-- 0x05BC
		when 000368 => D <= "--------------------------------";	-- 0x05C0
		when 000369 => D <= "--------------------------------";	-- 0x05C4
		when 000370 => D <= "--------------------------------";	-- 0x05C8
		when 000371 => D <= "--------------------------------";	-- 0x05CC
		when 000372 => D <= "--------------------------------";	-- 0x05D0
		when 000373 => D <= "--------------------------------";	-- 0x05D4
		when 000374 => D <= "--------------------------------";	-- 0x05D8
		when 000375 => D <= "--------------------------------";	-- 0x05DC
		when 000376 => D <= "--------------------------------";	-- 0x05E0
		when 000377 => D <= "--------------------------------";	-- 0x05E4
		when 000378 => D <= "--------------------------------";	-- 0x05E8
		when 000379 => D <= "--------------------------------";	-- 0x05EC
		when 000380 => D <= "--------------------------------";	-- 0x05F0
		when 000381 => D <= "--------------------------------";	-- 0x05F4
		when 000382 => D <= "--------------------------------";	-- 0x05F8
		when 000383 => D <= "--------------------------------";	-- 0x05FC
		when 000384 => D <= "--------------------------------";	-- 0x0600
		when 000385 => D <= "--------------------------------";	-- 0x0604
		when 000386 => D <= "--------------------------------";	-- 0x0608
		when 000387 => D <= "--------------------------------";	-- 0x060C
		when 000388 => D <= "--------------------------------";	-- 0x0610
		when 000389 => D <= "--------------------------------";	-- 0x0614
		when 000390 => D <= "--------------------------------";	-- 0x0618
		when 000391 => D <= "--------------------------------";	-- 0x061C
		when 000392 => D <= "--------------------------------";	-- 0x0620
		when 000393 => D <= "--------------------------------";	-- 0x0624
		when 000394 => D <= "--------------------------------";	-- 0x0628
		when 000395 => D <= "--------------------------------";	-- 0x062C
		when 000396 => D <= "--------------------------------";	-- 0x0630
		when 000397 => D <= "--------------------------------";	-- 0x0634
		when 000398 => D <= "--------------------------------";	-- 0x0638
		when 000399 => D <= "--------------------------------";	-- 0x063C
		when 000400 => D <= "--------------------------------";	-- 0x0640
		when 000401 => D <= "--------------------------------";	-- 0x0644
		when 000402 => D <= "--------------------------------";	-- 0x0648
		when 000403 => D <= "--------------------------------";	-- 0x064C
		when 000404 => D <= "--------------------------------";	-- 0x0650
		when 000405 => D <= "--------------------------------";	-- 0x0654
		when 000406 => D <= "--------------------------------";	-- 0x0658
		when 000407 => D <= "--------------------------------";	-- 0x065C
		when 000408 => D <= "--------------------------------";	-- 0x0660
		when 000409 => D <= "--------------------------------";	-- 0x0664
		when 000410 => D <= "--------------------------------";	-- 0x0668
		when 000411 => D <= "--------------------------------";	-- 0x066C
		when 000412 => D <= "--------------------------------";	-- 0x0670
		when 000413 => D <= "--------------------------------";	-- 0x0674
		when 000414 => D <= "--------------------------------";	-- 0x0678
		when 000415 => D <= "--------------------------------";	-- 0x067C
		when 000416 => D <= "--------------------------------";	-- 0x0680
		when 000417 => D <= "--------------------------------";	-- 0x0684
		when 000418 => D <= "--------------------------------";	-- 0x0688
		when 000419 => D <= "--------------------------------";	-- 0x068C
		when 000420 => D <= "--------------------------------";	-- 0x0690
		when 000421 => D <= "--------------------------------";	-- 0x0694
		when 000422 => D <= "--------------------------------";	-- 0x0698
		when 000423 => D <= "--------------------------------";	-- 0x069C
		when 000424 => D <= "--------------------------------";	-- 0x06A0
		when 000425 => D <= "--------------------------------";	-- 0x06A4
		when 000426 => D <= "--------------------------------";	-- 0x06A8
		when 000427 => D <= "--------------------------------";	-- 0x06AC
		when 000428 => D <= "--------------------------------";	-- 0x06B0
		when 000429 => D <= "--------------------------------";	-- 0x06B4
		when 000430 => D <= "--------------------------------";	-- 0x06B8
		when 000431 => D <= "--------------------------------";	-- 0x06BC
		when 000432 => D <= "--------------------------------";	-- 0x06C0
		when 000433 => D <= "--------------------------------";	-- 0x06C4
		when 000434 => D <= "--------------------------------";	-- 0x06C8
		when 000435 => D <= "--------------------------------";	-- 0x06CC
		when 000436 => D <= "--------------------------------";	-- 0x06D0
		when 000437 => D <= "--------------------------------";	-- 0x06D4
		when 000438 => D <= "--------------------------------";	-- 0x06D8
		when 000439 => D <= "--------------------------------";	-- 0x06DC
		when 000440 => D <= "--------------------------------";	-- 0x06E0
		when 000441 => D <= "--------------------------------";	-- 0x06E4
		when 000442 => D <= "--------------------------------";	-- 0x06E8
		when 000443 => D <= "--------------------------------";	-- 0x06EC
		when 000444 => D <= "--------------------------------";	-- 0x06F0
		when 000445 => D <= "--------------------------------";	-- 0x06F4
		when 000446 => D <= "--------------------------------";	-- 0x06F8
		when 000447 => D <= "--------------------------------";	-- 0x06FC
		when 000448 => D <= "--------------------------------";	-- 0x0700
		when 000449 => D <= "--------------------------------";	-- 0x0704
		when 000450 => D <= "--------------------------------";	-- 0x0708
		when 000451 => D <= "--------------------------------";	-- 0x070C
		when 000452 => D <= "--------------------------------";	-- 0x0710
		when 000453 => D <= "--------------------------------";	-- 0x0714
		when 000454 => D <= "--------------------------------";	-- 0x0718
		when 000455 => D <= "--------------------------------";	-- 0x071C
		when 000456 => D <= "--------------------------------";	-- 0x0720
		when 000457 => D <= "--------------------------------";	-- 0x0724
		when 000458 => D <= "--------------------------------";	-- 0x0728
		when 000459 => D <= "--------------------------------";	-- 0x072C
		when 000460 => D <= "--------------------------------";	-- 0x0730
		when 000461 => D <= "--------------------------------";	-- 0x0734
		when 000462 => D <= "--------------------------------";	-- 0x0738
		when 000463 => D <= "--------------------------------";	-- 0x073C
		when 000464 => D <= "--------------------------------";	-- 0x0740
		when 000465 => D <= "--------------------------------";	-- 0x0744
		when 000466 => D <= "--------------------------------";	-- 0x0748
		when 000467 => D <= "--------------------------------";	-- 0x074C
		when 000468 => D <= "--------------------------------";	-- 0x0750
		when 000469 => D <= "--------------------------------";	-- 0x0754
		when 000470 => D <= "--------------------------------";	-- 0x0758
		when 000471 => D <= "--------------------------------";	-- 0x075C
		when 000472 => D <= "--------------------------------";	-- 0x0760
		when 000473 => D <= "--------------------------------";	-- 0x0764
		when 000474 => D <= "--------------------------------";	-- 0x0768
		when 000475 => D <= "--------------------------------";	-- 0x076C
		when 000476 => D <= "--------------------------------";	-- 0x0770
		when 000477 => D <= "--------------------------------";	-- 0x0774
		when 000478 => D <= "--------------------------------";	-- 0x0778
		when 000479 => D <= "--------------------------------";	-- 0x077C
		when 000480 => D <= "--------------------------------";	-- 0x0780
		when 000481 => D <= "--------------------------------";	-- 0x0784
		when 000482 => D <= "--------------------------------";	-- 0x0788
		when 000483 => D <= "--------------------------------";	-- 0x078C
		when 000484 => D <= "--------------------------------";	-- 0x0790
		when 000485 => D <= "--------------------------------";	-- 0x0794
		when 000486 => D <= "--------------------------------";	-- 0x0798
		when 000487 => D <= "--------------------------------";	-- 0x079C
		when 000488 => D <= "--------------------------------";	-- 0x07A0
		when 000489 => D <= "--------------------------------";	-- 0x07A4
		when 000490 => D <= "--------------------------------";	-- 0x07A8
		when 000491 => D <= "--------------------------------";	-- 0x07AC
		when 000492 => D <= "--------------------------------";	-- 0x07B0
		when 000493 => D <= "--------------------------------";	-- 0x07B4
		when 000494 => D <= "--------------------------------";	-- 0x07B8
		when 000495 => D <= "--------------------------------";	-- 0x07BC
		when 000496 => D <= "--------------------------------";	-- 0x07C0
		when 000497 => D <= "--------------------------------";	-- 0x07C4
		when 000498 => D <= "--------------------------------";	-- 0x07C8
		when 000499 => D <= "--------------------------------";	-- 0x07CC
		when 000500 => D <= "--------------------------------";	-- 0x07D0
		when 000501 => D <= "--------------------------------";	-- 0x07D4
		when 000502 => D <= "--------------------------------";	-- 0x07D8
		when 000503 => D <= "--------------------------------";	-- 0x07DC
		when 000504 => D <= "--------------------------------";	-- 0x07E0
		when 000505 => D <= "--------------------------------";	-- 0x07E4
		when 000506 => D <= "--------------------------------";	-- 0x07E8
		when 000507 => D <= "--------------------------------";	-- 0x07EC
		when 000508 => D <= "--------------------------------";	-- 0x07F0
		when 000509 => D <= "--------------------------------";	-- 0x07F4
		when 000510 => D <= "--------------------------------";	-- 0x07F8
		when 000511 => D <= "--------------------------------";	-- 0x07FC
		when 000512 => D <= "00000000000000000000000000000000";	-- 0x0800
		when 000513 => D <= "00000000000000000000000000000001";	-- 0x0804
		when 000514 => D <= "11111111111111111111111111111111";	-- 0x0808
		when 000515 => D <= "01111111111111111111111111111111";	-- 0x080C
		when 000516 => D <= "10000000000000000000000000000000";	-- 0x0810
		when 000517 => D <= "00000000000000001010101111001101";	-- 0x0814
		when 000518 => D <= "00010010001101000101011001111000";	-- 0x0818
		when 000519 => D <= "11111110110111001011101010011000";	-- 0x081C
		when 000520 => D <= "00110110100100100101100000010100";	-- 0x0820
		when 000521 => D <= "00000000000000000000000000000000";	-- 0x0824
		when 000522 => D <= "00000000000000000000000000000000";	-- 0x0828
		when 000523 => D <= "00000000000000000000000000000000";	-- 0x082C
		when 000524 => D <= "10111011101110111011101110111011";	-- 0x0830
		when 000525 => D <= "10111011101110111011101110111011";	-- 0x0834
		when 000526 => D <= "10111011101110111011101110111011";	-- 0x0838
		when 000527 => D <= "10111011101110111011101110111011";	-- 0x083C
		when 000528 => D <= "11111111111111111111111111111111";	-- 0x0840
		when 000529 => D <= "11111111111111111111111111111111";	-- 0x0844
		when 000530 => D <= "11111111111111111111111111111111";	-- 0x0848
		when 000531 => D <= "11111111111111111111111111111111";	-- 0x084C
		when 000532 => D <= "11111111111111111111111111111111";	-- 0x0850
		when 000533 => D <= "11111111111111111111111111111111";	-- 0x0854
		when 000534 => D <= "11111111111111111111111111111111";	-- 0x0858
		when 000535 => D <= "11111111111111111111111111111111";	-- 0x085C
		when 000536 => D <= "11111111111111111111111111111111";	-- 0x0860
		when 000537 => D <= "11111111111111111111111111111111";	-- 0x0864
		when 000538 => D <= "11111111111111111111111111111111";	-- 0x0868
		when 000539 => D <= "11111111111111111111111111111111";	-- 0x086C
		when 000540 => D <= "11111111111111111111111111111111";	-- 0x0870
		when 000541 => D <= "11111111111111111111111111111111";	-- 0x0874
		when 000542 => D <= "11111111111111111111111111111111";	-- 0x0878
		when 000543 => D <= "11111111111111111111111111111111";	-- 0x087C
		when 000544 => D <= "11111111111111111111111111111111";	-- 0x0880
		when 000545 => D <= "11111111111111111111111111111111";	-- 0x0884
		when 000546 => D <= "11111111111111111111111111111111";	-- 0x0888
		when 000547 => D <= "11111111111111111111111111111111";	-- 0x088C
		when 000548 => D <= "11111111111111111111111111111111";	-- 0x0890
		when 000549 => D <= "11111111111111111111111111111111";	-- 0x0894
		when 000550 => D <= "11111111111111111111111111111111";	-- 0x0898
		when 000551 => D <= "11111111111111111111111111111111";	-- 0x089C
		when 000552 => D <= "11111111111111111111111111111111";	-- 0x08A0
		when 000553 => D <= "11111111111111111111111111111111";	-- 0x08A4
		when 000554 => D <= "11111111111111111111111111111111";	-- 0x08A8
		when 000555 => D <= "11111111111111111111111111111111";	-- 0x08AC
		when 000556 => D <= "11111111111111111111111111111111";	-- 0x08B0
		when 000557 => D <= "11111111111111111111111111111111";	-- 0x08B4
		when 000558 => D <= "11111111111111111111111111111111";	-- 0x08B8
		when 000559 => D <= "11111111111111111111111111111111";	-- 0x08BC
		when 000560 => D <= "11111111111111111111111111111111";	-- 0x08C0
		when 000561 => D <= "11111111111111111111111111111111";	-- 0x08C4
		when 000562 => D <= "11111111111111111111111111111111";	-- 0x08C8
		when 000563 => D <= "11111111111111111111111111111111";	-- 0x08CC
		when 000564 => D <= "11111111111111111111111111111111";	-- 0x08D0
		when 000565 => D <= "11111111111111111111111111111111";	-- 0x08D4
		when 000566 => D <= "11111111111111111111111111111111";	-- 0x08D8
		when 000567 => D <= "11111111111111111111111111111111";	-- 0x08DC
		when 000568 => D <= "11111111111111111111111111111111";	-- 0x08E0
		when 000569 => D <= "11111111111111111111111111111111";	-- 0x08E4
		when 000570 => D <= "11111111111111111111111111111111";	-- 0x08E8
		when 000571 => D <= "11111111111111111111111111111111";	-- 0x08EC
		when 000572 => D <= "11101110111011101110111011101110";	-- 0x08F0
		when 000573 => D <= "11101110111011101110111011101110";	-- 0x08F4
		when 000574 => D <= "11101110111011101110111011101110";	-- 0x08F8
		when 000575 => D <= "11101110111011101110111011101110";	-- 0x08FC
		when others => D <= "--------------------------------";
		end case;
	end process;
end;
