MCU: STM32F411CEU6  
Platform: Clion & STM32CubeMX
Debugger: DapLink

CAN Wire Connect:   
PA4 - SPI1 CS  
PA5 - SPI1 CLK  
PA6 - SPI1 MISO  
PA7 - SPI1 MOSI  
PB0 - MCP2517FD INT0  
PB1 - MCP2517FD INT1   

Other Wire  (can ignore those, some hardware resource just for debug on board)
PA0 - Red Led (low level active)  
PA1 - Green Led (low level active)  
PA2 - Blue Led (low level active)  

---
TIM10: Auto Send CAN2.0 Frame, 5K Hz  

---
MCP2517FD:  
SYSCLK: 20M Osc on board
---  
### NOTE:  
Official library(MicroChip MCP251xFD) have been modified by this in DRV_CANFDSPI_TransmitChannelLoad 
function, I comment those lines :
```c++
    // Check that it is a transmit buffer
    ciFifoCon.word = fifoReg[0];
    if (!ciFifoCon.txBF.TxEnable) {
        return -2;
    }
```
