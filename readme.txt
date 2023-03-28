这是qspi flash 应用程序流水灯测试：
注意事项：
1. 在main函数第一行加入SCB->VTOR = 0x90000000; /* 设置中断向量表地址 */
2. 注释掉MPU_Config
3. 不要修改和qspi有关的任何参数
4. HCLK3必须维持在120MHz
