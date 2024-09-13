.data
    n: .word 7
    
.text
.globl __start

FUNCTION:
addi x31, x0, 1 #give x31 the value of 2
addi x12, x0, 8  # x12 = = 8
addi x13, x0, 4
addi x2, x2, -16
sw x1, 8(x2)
jal x1, funct_recur
addi x10, x5, 0
lw x1, 8(x2)
jalr x0, 0(x1)
funct_recur: #using the method provided in the textbook

    addi x2, x2, -16
    sw x1, 8(x2)
    sw a0, 0(x2)
    #bge a0, x31, loop  #if n is greater than 2, go to loop
    beq a0, x31, loop2
    beq x0, x0, loop
loop2:
    addi t0, x0, 7  #else return 7
    addi x2, x2, 16
    jalr x0, 0(x1)

loop:
    srli a0, a0, 1  #a0=floor(a0/2)
    jal x1, funct_recur
    addi x6, t0, 0  #move T(floor(n/2)) to x6
    mul x6,x6,x12  #x6=x6*8
    lw a0, 0(x2)
    lw x1, 8(x2)
    mul x11, a0, x13  #x11=a0*4
    add t0, x6, x11  #Store the result T(n) into t0, as desired.
    addi x2, x2, 16
    jalr x0, 0(x1)

exit:
   

# Do NOT modify the following
__start:
    la   t0, n
    lw   x10, 0(t0)
    jal  x1,FUNCTION
    la   t0, n
    sw   x10, 4(t0)
    addi a0,x0,10
    ecall