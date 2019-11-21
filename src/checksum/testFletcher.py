'''
Created on 4 nov. 2019

@author: DECRAEMK
'''
from checksum.fletcherNbit import Fletcher

if __name__ == '__main__':

    # 0x7f 0000000019c90013004e000001790000 3190
    # 0x7f 0000000019ca0013004e000001790000 259b
    # 0x7f 0000000019cb0013004e000001790000 19a6
    # 0x7f 0000000019cc0013004e000001790000 0db1
    # 0x7f 0000000019cd0013004e000001790000 01bc
    # 0x7f 0000000019ce0013004e000001790000 f4c7
    # 0x7f 0000000019cf0013004e000001790000 e8d2
    # 0x7f 0000000019d00013004e000001790000 dcdd
    # 0x7f 0000000019d10013004e000001790000 d0e8
    # 0x7f 0000000019d20013004e000001790000 c4f3
    # 0x7f 0000000019d30013004e000001790000 b8fe
    # 0x7f 0000000019d40013004e000001790000 ac0e
    # 0x7f 0000000019d50013004e000001790000 a015
    # 0x7f 000000001c67001500530000017c0000 5eb8

    samples = [(bytearray.fromhex("7f0000000019c90013004e000001790000"),0x3190),
               (bytearray.fromhex("7f0000000019cb0013004e000001790000"),0x19a6),
               (bytearray.fromhex("7f0000000019cc0013004e000001790000"),0x0db1),
               (bytearray.fromhex("7f0000000019cd0013004e000001790000"),0x01bc),
               (bytearray.fromhex("7f0000000019ce0013004e000001790000"),0xf4c7),
               (bytearray.fromhex("7f0000000019cf0013004e000001790000"),0xe8d2),
               (bytearray.fromhex("7f0000000019d00013004e000001790000"),0xdcdd),
               (bytearray.fromhex("7f0000000019d10013004e000001790000"),0xd0e8),
               (bytearray.fromhex("7f0000000019d20013004e000001790000"),0xc4f3),
               (bytearray.fromhex("7f0000000019d30013004e000001790000"),0xb8fe),
               (bytearray.fromhex("7f0000000019d40013004e000001790000"),0xac0e),
               (bytearray.fromhex("7f0000000019d50013004e000001790000"),0xa015),
               (bytearray.fromhex("7f000000001c67001500530000017c0000"),0x5eb8),
#                (bytearray.fromhex("000a108003ed07090001004000f802fe04"),0x2104),
#                (bytearray.fromhex("130331fc02fb001fffff03fc0af804fe0e"),0x0c7e)
               ]
         
    for sample in samples:
        f8 = Fletcher()
        f8.update(sample[0])
        result = (f8.cb0 << 8) + f8.cb1
        if result == sample[1]:
            print(sample[0].hex() + " " + hex(result) + " PASS")
        else:
            print(sample[0].hex() + " " + hex(result) + " FAIL")
