/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mbed.h"
#include "ble/BLE.h"

#include "ble/services/UARTService.h"
#include <ctime>
#include <cstdio>

BLEDevice  ble;
DigitalOut led1(LED1);
Timer t;
Serial pc(USBTX, USBRX);

UARTService *uartServicePtr;

uint32_t iterations = 1;
uint32_t _Recv[8];
uint32_t loop_array[8];
struct range {
    const uint32_t *data; //restrict
    uint32_t address;
    size_t size;
};

void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    pc.printf("Disconnected!/n/r");
    pc.printf("Restarting the advertising process\n\r");
    ble.startAdvertising();
}

/* rotate left bits
 * Rotates the bit pattern of reg left by amt.
 */
static inline uint32_t rotl(uint32_t reg, uint32_t amt)
{
    return reg << amt | (reg >> (32 - amt));
}
/* rotate right bits
 * Rotates the bit pattern of reg right by amt.
 */
static inline uint32_t rotr(uint32_t reg, uint32_t amt)
{
    return rotl(reg, 32 - amt);
}
/* load register
 * Mimics the behaviour of the ARM ldr instruction on a simulated memory.
 */
static inline uint32_t ldr(uint32_t addr, struct range mem)
{
    mem.data = mem.data + ((addr - mem.address) >> 2);
    char const *src = (char const *) mem.data;
    uint32_t result = src[3] << 24 | src[2] << 16 | src[1] << 8 | src[0] << 0;
    return rotr(result, ((addr - mem.address) & 3) * 8);
}

void checkfn_c(
    uint32_t y,
    uint32_t *C, //restrict
    const struct range *regions, //restrict
    uint32_t region_count,
    uint32_t lr,
    const struct range checksum) 
{
    #define Cj(n) C[(j - n) & 7]
    uint32_t i, j, x, s, d, val, e;
    const struct range *range;

    s = 0x000000d3;
    j = 0;
    x = lr;
    range = &checksum;
    
    do {
        e = 0;
        lr = checksum.address + 0x478;
        for (i = 0; i <= y; i++) {
                switch ((Cj(0) >> 4) & 3) {
                    case 0:
                    
                    x = (x + ((x*x) | 5)) % 32;
                    s &= ~0xc0000000;
                    s |= (x & 0x80000000);
                    s |= x == 0 ? 0x40000000 : 0;
//                    Cj(0) |= 0xf; 
                    x = x ^ rotl(Cj(0), 4);
                    d = range->address + (x % range->size);
                    val = ldr(d, *range);
                    Cj(0) = (rotr(Cj(1), 2)  ^ Cj(7)) + e;
                    Cj(1) = (rotr(lr, 4)     ^ Cj(0)) + Cj(2);
                    Cj(2) = (rotr(Cj(3), 6)  ^ Cj(1)) + d;
                    Cj(3) = (rotr(Cj(4), 8)  ^ Cj(2)) + (checksum.address + 0x43c);
                    Cj(4) = (rotr(s, 10)     ^ Cj(3)) + Cj(5);
                    Cj(5) = (rotr(Cj(6), 12) ^ Cj(4)) + val;
                    Cj(6) = (rotr(Cj(2), 14) ^ Cj(5)) + Cj(7);
                    Cj(7) = (rotr(i, 16)     ^ Cj(6)) + Cj(0);
                    Cj(0) = (Cj(0) ^ lr) + val;
//                    Cj(0) |= 0xf; 
                    e = ldr(checksum.address + 0x478 + (((int32_t)x) >> 22), checksum);
                    lr = (checksum.address + 0x478);
                    
                    break;
                    
                    case 1:
                    x = (x + ((x*x) | 5)) % 32;
                    s &= ~0xc0000000;
                    s |= (x & 0x80000000);
                    s |= x == 0 ? 0x40000000 : 0;
                    x = x ^ rotl(Cj(3), 8);
                    d = range->address + (x % range->size);
                    val = ldr(d, *range);
                    Cj(4) = (rotr(lr, 18)    ^ Cj(3)) + e;
                    Cj(5) = (rotr(Cj(6), 20) ^ Cj(4)) + d;
                    Cj(6) = (rotr(Cj(2), 22) ^ Cj(5)) + Cj(7);
                    Cj(7) = (rotr(s, 24)     ^ Cj(6)) + Cj(0);
                    Cj(0) = (rotr(Cj(1), 26) ^ Cj(7)) + Cj(6);
                    Cj(1) = (rotr(i, 28)     ^ Cj(0)) + Cj(2);
                    Cj(2) = (rotr(Cj(3), 30) ^ Cj(1)) + val;
                    Cj(3) = (rotr(Cj(4), 4)  ^ Cj(2)) + (checksum.address + 0x4f8);
                    Cj(2) = (Cj(2) ^ lr) + val;
                    e = ldr(checksum.address + 0x500 + (((int32_t)x) >> 22), checksum);
                    lr = (checksum.address + 0x500);
                    
                    break;
                    
                    case 2:
                    x = (x + ((x*x) | 5)) % 32;
                    s &= ~0xc0000000;
                    s |= (x & 0x80000000);
                    s |= x == 0 ? 0x40000000 : 0;
                    x = x ^ (rotl(Cj(4), 12));
                    d = range->address + (x % range->size);
                    val = ldr(d, *range);
                    Cj(6) = (rotr(Cj(2), 8)  ^ Cj(5)) + e;
                    Cj(7) = (rotr(lr, 12)    ^ Cj(6)) + Cj(0);
                    Cj(4) = (rotr(d, 16)     ^ Cj(3)) + Cj(5);
                    Cj(5) = (rotr(Cj(6), 20) ^ Cj(4)) + s;
                    Cj(2) = (rotr(Cj(3), 24) ^ Cj(1)) + val;
                    Cj(3) = (rotr(Cj(4), 28) ^ Cj(2)) + (checksum.address + 0x55c);
                    Cj(0) = (rotr(Cj(1), 2)  ^ Cj(7)) + Cj(6);
                    Cj(1) = (rotr(i, 6)      ^ Cj(0)) + Cj(2);
                    Cj(1) = (Cj(1) ^ lr) + val;
                    e = ldr(checksum.address + 0x588 + (((int32_t)x) >> 22), checksum);
                    lr = (checksum.address + 0x588);
                   
                    break;
                    
                    default:
                    x = (x + ((x*x) | 5)) % 32; 
                    s &= ~0xc0000000;
                    s |= (x & 0x80000000);
                    s |= x == 0 ? 0x40000000 : 0;
                    x = x ^ rotl(Cj(6), 16);
                    d = range->address + (x % range->size);
                    val = ldr(d, *range);
                    Cj(7) = (rotr(Cj(6), 10) ^ lr)    + e;
                    Cj(6) = (rotr(Cj(5), 14) ^ Cj(2)) + Cj(7);
                    Cj(5) = (rotr(Cj(4), 18) ^ Cj(6)) + d;
                    Cj(4) = (rotr(Cj(3), 22) ^ Cj(4)) + Cj(5);
                    Cj(3) = (rotr(Cj(2), 26) ^ Cj(4)) + (checksum.address + 0x5d8);
                    Cj(2) = (rotr(Cj(1), 30) ^ Cj(3)) + val;
                    Cj(1) = (rotr(Cj(0), 8)  ^ Cj(2)) + s;
                    Cj(0) = (rotr(Cj(7), 16) ^ Cj(6)) + Cj(1);
                    Cj(7) = (Cj(7) ^ lr) + val;
                    e = ldr(checksum.address + 0x610 + (((int32_t)x) >> 21), checksum);
                    lr = (checksum.address + 0x610);
                    
                    break;
                }
                j = (j + 1) % 8;
                s |= 0x20000000; // carry bit from --i
        }
        range = regions++;
    } while (region_count--);
#undef Cj
}


void onDataWritten(const GattWriteCallbackParams *params)
{
    if ((uartServicePtr != NULL) && (params->handle == uartServicePtr->getTXCharacteristicHandle())) {
        uint16_t bytesRead = params->len;

        char * rec = (char *)params->data; //casting to char

        for(int i = 0; i < 8; i++) {
            _Recv[i] = (uint32_t) *rec++ ;
        }

        range asm_f;
        asm_f.data = (uint32_t*) &checkfn_c;
        asm_f.address = (uint32_t) &checkfn_c;
        asm_f.size = 0x730;

        //for(int i = 0; i < 8; i++) {
            //pc.printf("data before attestation: %u \n\r", _Recv[i]);
        //}
        
        t.start();
        int begin, end, tot_time = 0, curr_time;
        /*for(int i = 0; i < 100; i++){
            for(int j = 0; j < 8; j++){
                loop_array[j] = _Recv[j];
            }    
            begin = t.read_us();
            checkfn_c(iterations, loop_array, NULL, 0, 0, asm_f);
            end = t.read_us();
            curr_time = end - begin;
            pc.printf("time: %u \n\r", curr_time); 
            tot_time = tot_time + curr_time;    
        }
        t.stop; 
        pc.printf("final time: %u \n\r", tot_time/100);*/ 
        checkfn_c(iterations, loop_array, NULL, 0, 0, asm_f);
        end = t.read_us();
        t.stop();
        curr_time = end - begin;
        pc.printf("time: %u \n\r", curr_time);
        //for(int i = 0; i < 8; i++) {
            //pc.printf("attested data: %u \n\r", _Recv[i]);
        //}

     //   uint8_t * test = (uint8_t *) _Recv; //convertin to uint8_t
     //   for(int i = 0; i < 8; i += 4) {
     //       printf("test data: %u \n\r", (test[i+3] << 24 | test[i+2] << 16 | test[i+1] << 8 | test[i] << 0));
     //   }

        ble.updateCharacteristicValue(uartServicePtr->getRXCharacteristicHandle(), (uint8_t *) _Recv, bytesRead);
    }
}

void periodicCallback(void)
{
    led1 = !led1;
}

int main(void)
{
    led1 = 1;
    Ticker ticker;
    ticker.attach(periodicCallback, 1);

    pc.printf("Initialising the nRF51822\n\r");
    ble.init();
    ble.onDisconnection(disconnectionCallback);
    ble.onDataWritten(onDataWritten);

    /* setup advertising */
    ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED);
    ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME,
                                     (const uint8_t *)"BLE UART", sizeof("BLE UART") - 1);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                     (const uint8_t *)UARTServiceUUID_reversed, sizeof(UARTServiceUUID_reversed));

    ble.setAdvertisingInterval(1000); /* 1000ms; in multiples of 0.625ms. */
    ble.startAdvertising();

    UARTService uartService(ble);
    uartServicePtr = &uartService;

    while (true) {
        ble.waitForEvent();
    }
}
