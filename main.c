#include <hidef.h> /* common defines and macros */
#include "derivative.h" /* derivative-specific definitions */
#include <stdio.h>
#include <stdlib.h>

/* PORTA
 * PA0 - a
 * PA1 - b
 * ...
 * PA6 - g
 * PA7 - dp
 *
 * 0 - b 0011 1111 = 0x3f
 * 1 - b 0000 0110 = 0x06
 * 2 - b 0101 1011 = 0x5b
 * 3 - b 0100 1111 = 0x4f
 * 4 - b 0110 0110 = 0x66
 * 5 - b 0110 1101 = 0x6d
 * 6 - b 0111 1101 = 0x7d
 * 7 - b 0000 0111 = 0x07
 * 8 - b 0111 1111 = 0x7f
 * 9 - b 0110 1111 = 0x6f
 * dp - b 1000 0000 = 0x80
 */

#define zeroHex 0x3f
#define oneHex 0x06
#define twoHex 0x5b
#define threeHex 0x4f
#define fourHex 0x66
#define fiveHex 0x6d
#define sixHex 0x7d
#define sevenHex 0x07
#define eightHex 0x7f
#define nineHex 0x6f
#define dpOnHex 0x80

#define maxSize 10 // queue max size
#define stpMtrDelay for(i = 0; i < 10000; i++) // stepper motor delay

int btnQueue[maxSize], destQueue[maxSize];

unsigned char LVL_ONE, LVL_TWO, LVL_THREE, ROT_SPD, PORTA_outval, PORTB_outval;
unsigned char PRIsHighSignal, PRHighFlag, PRLowFlag;
unsigned char dir, spd, data, tmp;

unsigned int btnQueueIndex, destQueueIndex, lvlPartCnt, currLvl, dest;

unsigned long i, j;

unsigned char convertIntToBcdHex(int num){
    unsigned char bcdHex;

    switch(num){
        case 0:
            bcdHex = zeroHex;
            break;
        case 1:
            bcdHex = oneHex;
            break;
        case 2:
            bcdHex = twoHex;
            break;
        case 3:
            bcdHex = threeHex;
            break;
        case 4:
            bcdHex = fourHex;
            break;
        case 5:
            bcdHex = fiveHex;
            break;
        case 6:
            bcdHex = sixHex;
            break;
        case 7:
            bcdHex = sevenHex;
            break;
        case 8: 
            bcdHex = eightHex;
            break;
        case 9:
            bcdHex = nineHex;
            break;
        default:
            break;
    }

    return bcdHex;
}

void rotateMotor(unsigned char spd, unsigned char dir){
    
    unsigned char PORTB_outval;
    
    const unsigned char full[4] = {0x36, 0x35, 0x39, 0x3A}; // 90 degrees
    const unsigned char half[8] = {0x32, 0x36, 0x34, 0x35, 0x31, 0x39, 0x38, 0x3A}; // 45 degrees
    
    switch(spd){
        case 0:
            if(dir){
                /*for(j = 3; j >= 0; j--){
                PORTB_outval = full[j];
                PORTB = PORTB_outval;
                for(i = 0; i < 10000; i++);
                }*/
                // reverse
                PORTB_outval = 0x3A;
                PORTB = PORTB_outval;
                stpMtrDelay;
                PORTB_outval = 0x39;
                PORTB = PORTB_outval;
                stpMtrDelay;
                PORTB_outval = 0x35;
                PORTB = PORTB_outval;
                stpMtrDelay;
                PORTB_outval = 0x36;
                PORTB = PORTB_outval;
                stpMtrDelay;
            } 
            else if(!dir){
                for(j = 0; j <= 3; j++){
                    PORTB_outval = full[j];
                    PORTB = PORTB_outval;
                    stpMtrDelay;
                }
            }
            break;
        case 1:
            if(dir){
                /*for(j = 7; j >= 0; j--){
                PORTB_outval = half[j];
                PORTB = PORTB_outval;
                for(i = 0; i < 10000; i++);
                }*/
                // reverse
                PORTB_outval = 0x3A;
                PORTB = PORTB_outval;
                stpMtrDelay;
                PORTB_outval = 0x38;
                PORTB = PORTB_outval;
                stpMtrDelay;
                PORTB_outval = 0x39;
                PORTB = PORTB_outval;
                stpMtrDelay;
                PORTB_outval = 0x31;
                PORTB = PORTB_outval;
                stpMtrDelay;
                PORTB_outval = 0x35;
                PORTB = PORTB_outval;
                stpMtrDelay;
                PORTB_outval = 0x34;
                PORTB = PORTB_outval;
                stpMtrDelay;
                PORTB_outval = 0x36;
                PORTB = PORTB_outval;
                stpMtrDelay;
                PORTB_outval = 0x32;
                PORTB = PORTB_outval;
                stpMtrDelay;
            } 
            else if(!dir){
                for(j = 0; j <= 7; j++){
                    PORTB_outval = half[j];
                    PORTB = PORTB_outval;
                    stpMtrDelay;
                }
            }
            break;
        default:
            break;
    }
}

void initAnalog(){
    //printf("entered initAnalog!\r\n");
    
    // ATD power on, ATD fast flag clear bit
    ATD0CTL2 = 0xC0; // Turn on A/D converter;
    for (i = 0; i < 150000; i++); // Wait at least 100 milliseconds for initialization;
    
    // Configure A/D for proper operation;
    ATD0CTL3 = 0x08; // 1 conversion
    ATD0CTL4 = 0xE0; // 0x60 for 10-bit, 0xE0 for 8-bit
    
    //printf("finished initAnalog!\r\n");
}

char PRIsHigh(){
    //printf("entered PRIsHigh!\r\n");
    while (!(ATD0STAT0 & 0x80)); // Monitor for A-D conversion completion in status register;
    data = ATD0DR0; // Get the results in the corresponding results register;
    
    if(data > 19){
      //printf("PRIsHigh: high\r\n");
      return 1;
    }
    else{
      //printf("PRIsHigh: low\r\n");
      return 0;
    }
    return 0;
} 

void initDataDirection(){
    DDRA = 0xff;
    DDRB = 0xff;
    DDRK = 0x00;
}

void printArr(int * arr){
    for(i = 0; i < maxSize / 2; i++){
      printf("arr[%u]: %d\r\n", (unsigned int)i, *(arr++) );
    }
    printf("\r\n");
}

// duplicating an array: http://stackoverflow.com/a/31703609
void cstringcpy(int * dest, int * src)
{
    //printf("\r\nsrc: ");
    //printArr(src);
    
    // clear dest
    /*for(i = 0; i < maxSize; i++){
      dest[i] = 0; 
    }*/
    
    while (*(src)) {
        *(dest++) = *(src++);
    }   
}

int arrSize = 0;
int sizeOfArr(int *arr){
    //printf("arr before sizeOfArr: %d ... ", arr);
    // find size of arr
    arrSize = 0;
    //printf("finding size of arr\r\n");
    while(*(arr++)){
        //printf("arr[%d]: %d\r\n", arrSize, *(arr++));
        arrSize++;
    }
    printf("size of arr: %d\r\n", arrSize);
    //printf("arr after sizeOfArr: %d ... ", arr);
    return arrSize;
}

/* Bubble Sort - http://www.studystreet.com/c-program-sort-array-ascending-order/
 * arr[] - input array
 * ascOrder
 *  0 - descending order
 *  1 - ascending order
 */
void bubbleSort(int * arr, int ascOrder){
    //printf("entered bubbleSort!\r\n");
    
    //printf("ascOrder: %d ... ", ascOrder);
    
    arrSize = sizeOfArr(arr);
    
    //printf("arr outside and after sizeOfArr: %d ... ", arr);
    
    //printf("arr: ");
    //printArr(arr);

    cstringcpy(destQueue, arr);
    
    //printf("destQueue: ");
    //printArr(destQueue); 
      
    //printf("destQueue after cstringcpy: ");
    //printArr(destQueue);
        
    for(i = 0; arrSize > 1 && i < arrSize; i++) {
        for(j = 0; j < arrSize - i; j++) {
            if(  (ascOrder && destQueue[j] != 0 && destQueue[j+1] != 0 && destQueue[j] > destQueue[j+1]) ||
                (!ascOrder && destQueue[j] != 0 && destQueue[j+1] != 0 && destQueue[j] <= destQueue[j+1]) ) { // dir = 1 = asc
                tmp = destQueue[j];
                destQueue[j] = destQueue[j+1];
                destQueue[j+1] = tmp;
            }
        }
    }
    
    destQueueIndex = 0;   
    printf("destQueue after sorting: w/ destQueueIndex: %d\r\n", destQueueIndex);
    printArr(destQueue);	    
     
    //printf("finished bubbleSort!\r\n");
}

enum btn_states {btn_init, btn_wait, btn_one, btn_two, btn_three, btn_wait2} btn_state;
void readBtnInput(){
    //printf("entered readBtnInput!\r\n");
	switch(btn_state){ //transitions
		case -1:
			btn_state = btn_init;
			break;
		case btn_init:
			btn_state = btn_wait;
			break;
		case btn_wait:
			if(!LVL_ONE && LVL_TWO && LVL_THREE){ // level one pressed
				btn_state = btn_one;
			}
			else if(LVL_ONE && !LVL_TWO && LVL_THREE){ // level two pressed
				btn_state = btn_two;
			}
			else if(LVL_ONE && LVL_TWO && !LVL_THREE){ // level three pressed
				btn_state = btn_three;
			}
			else{
				btn_state = btn_wait;
			}
			break;
		case btn_one:
			btn_state = btn_wait2;
			break;
		case btn_two:
			btn_state = btn_wait2;
			break;
		case btn_three:
			btn_state = btn_wait2;
			break;
		case btn_wait2:
			if(!LVL_ONE || !LVL_TWO || !LVL_THREE){
				btn_state = btn_wait2;
			}
			else{
				btn_state = btn_wait;
			}
			break;
		default:
			btn_state = btn_init;
			break;
	}
	//printf("finshed first switch statement\r\n");
	switch(btn_state){ //actions
		case btn_init:
		    break;
		case btn_wait:
			break;
		case btn_one:
            printf("btn_ONE pressed!\r\n");
            btnQueue[btnQueueIndex++] = 1;
            printf("btnQueue: w/ btnQueueIndex: %d\r\n", btnQueueIndex);
            printArr(btnQueue);
			break;
		case btn_two:
			printf("btn_TWO pressed!\r\n");
			printf("lvlPartCnt: %d ... ", lvlPartCnt);
			printf("currLvl: %d ... ", currLvl);
			
	    if(lvlPartCnt < abs(currLvl - 2) * 3){
            printf("valid button press! i can squeeze you in!\r\n");
            // directly enqueue onto top of destQueue
            if(sizeOfArr(destQueue) < maxSize){ // if there is at least one spot left on the queue
                //printf("destQueue has space\r\n");
                destQueue[sizeOfArr(destQueue)] = 2;
                //printf("dir: %d ... ", dir);
                bubbleSort(destQueue, dir); 
                // dest (destination) is defined as destQueue[destQueueIndex], initially the 0th element
                dest = destQueue[destQueueIndex++];
                printf("new dest: %d\r\n", dest);
            }
        }
        else{
            btnQueue[btnQueueIndex++] = 2;
            printArr(btnQueue);
        }
			break;
		case btn_three:
			printf("btn_THREE pressed!\r\n");
      btnQueue[btnQueueIndex++] = 3;
      printf("btnQueue: w/ btnQueueIndex: %d\r\n", btnQueueIndex);
      printArr(btnQueue);
			break;
		case btn_wait2:
			break;
		default:
			break;
	}
    //printf("finshed second switch statement\r\n");
    //printf("exited readBtnInput!\r\n");
}

void clearBtnQueue(){
    // init/clear btnQueue
    btnQueueIndex = 0;
    for (i = 0; i < maxSize; i++) {
        btnQueue[i] = 0;
    }
}

void clearDestQueue(){
    // init/clear btnQueue
    destQueueIndex = 0;
    for (i = 0; i < maxSize; i++) {
        destQueue[i] = 0;
    }
}

char queueIsEmpty(int * arr){
    for(i = 0; i < maxSize; i++){
        if( *(arr++) != 0 ){
            return 0;
        }
    }
    return 1;
}

void main(void) {

    clearBtnQueue();
    clearDestQueue();
  
    initDataDirection();
    initAnalog();
  
    PORTA_outval = 0x00;  // BCD
    PORTA = PORTA_outval;
  
    PORTB_outval = 0x30; // motor
    PORTB = PORTB_outval;
    
    // init so can detect first PRLowFlag
    PRHighFlag = 1;
    //PRLowFlag = 0;
    
    dir = spd = 0;
    
    i = j = 0;
    
    btn_state = -1;
    
    lvlPartCnt = 0;
    currLvl = 1;
    
    btnQueueIndex = destQueueIndex = 0;
    
    
       
    while(1){
        ATD0CTL5 = 0x85; // right-justified, AN5
        
        PORTB_outval = 0x30;
        PORTB = PORTB_outval;
        
        ROT_SPD = PORTK & 0x02;
        
        // level input
        LVL_ONE = PORTK & 0x04;
        LVL_TWO = PORTK & 0x08;
        LVL_THREE = PORTK & 0x10;
        
        readBtnInput();
        
        /*if(ROT_SPD == 0x02){
            spd = 0; // 45 degrees
        } else {
            spd = 1; // 90 degrees
        }*/
    
        // check if destQueue is empty
        if(queueIsEmpty(destQueue) && queueIsEmpty(btnQueue)){
            //printf("queues are empty\r\n");
            continue; // dont execute the rest of this while loop iteration
        }
        
        // choose level(s) to go to
        if(currLvl == 1 && queueIsEmpty(destQueue)){ // only direction is up. choose based on lowest (closest) level
            // sort btnQueue by ascending (lowest first)
            // iterate through all levels up before traversing a new btnQueue
              
            printf("currLvl = 1\r\n");
            bubbleSort(btnQueue, 1);
            // dest (destination) is defined as destQueue[destQueueIndex], initially the 0th element
            dest = destQueue[destQueueIndex++];
            printf("dest: %d\r\n", dest);
        
            clearBtnQueue();
        }
        else if(currLvl == 2 && queueIsEmpty(destQueue)){ // direction is either. choose based on priority
            // iterate through based on priority
            // do not touch btnQueue
            // continue
        
            printf("currLvl = 2\r\n");
            
            // assign destQueue to a new btnQueue
            cstringcpy(destQueue, btnQueue);
            destQueueIndex = 0;
            // dest (destination) is defined as destQueue[destQueueIndex], initially the 0th element
            dest = destQueue[destQueueIndex++];
            printf("dest: %d\r\n", dest);
            clearBtnQueue();
            
        }
        else if(currLvl == 3 && queueIsEmpty(destQueue)){ // only direction is up. choose based on lowest (closest) level
            // sort btnQueue by descending (highest first)
            // iterate through all levels down before traversing a new btnQueue

            printf("currLvl = 3\r\n");

            bubbleSort(btnQueue, 0);
            // dest (destination) is defined as destQueue[destQueueIndex], initially the 0th element
            dest = destQueue[destQueueIndex++];
            printf("dest: %d\r\n", dest);
            clearBtnQueue();
        }
       
        // set dir of motor 
        if(currLvl > dest){ // down = cc????
            dir = 0;
        }
        else if(currLvl < dest){ // up = ccw????
            dir = 1;
        }
        
        PRIsHighSignal = PRIsHigh();
    
        if(PRIsHighSignal && !PRHighFlag/* && PRLowFlag*/){
            PRHighFlag = 1;
            //PRLowFlag = 0;
            
            lvlPartCnt++;
            
            printf("lvlPartCnt++\r\n");
        }
        else if(!PRIsHighSignal && PRHighFlag /*&& !PRLowFlag*/){
            PRHighFlag = 0;
            //PRLowFlag = 1;
            
            lvlPartCnt++;
            
            printf("lvlPartCnt++\r\n");
        }
        
        // each level is defind as three level parts (lvlPartCnt) - low, high, low = 3 counts
        if(lvlPartCnt != 0 && lvlPartCnt % 3 == 0){ // completed moving to a different level
            printf("lvlPartCnt: %d\r\n", lvlPartCnt);
            if(dir)
                currLvl++;
            else if(!dir)
                currLvl--;
            
            // allow detection of initial low PRIsHigh signal and incr initial lvlPartCnt
            PRHighFlag = 1;
            //PRLowFlag = 0;
            
            printf("currLvl = %d\r\n", currLvl);
            
            // update BCD with currLvl
            PORTA_outval = convertIntToBcdHex(currLvl);
            PORTA = PORTA_outval;
        }

        if(currLvl != dest) { // hasn't reached dest yet - rotate motor
            printf("spd: %d, dir: %d\r\n", spd, dir);
            rotateMotor(spd, dir);
        }
        if(currLvl == dest){ // reached dest - don't rotate motor
            //printf("currLvl = dest\r\n");
            
            printf("destQueue: w/ destQueueIndex: %d\r\n", destQueueIndex);
            printArr(destQueue);
            
            if(destQueueIndex < sizeOfArr(destQueue) /*destQueue[destQueueIndex] != 0*/) { // next dest is valid
                // set dest = next dest on destQueue

                dest = destQueue[destQueueIndex++];
                
                //printf("destQueue @ index: %d\n", destQueueIndex);
                //printArr(destQueue);
            }
            else{ // next dest is not valid (0)
                printf("destQueue cleared, lvlPartCnt zeroed, and flags cleared\r\n");
                clearDestQueue();
                lvlPartCnt = 0;
                PRHighFlag = 1;
                //PRLowFlag = 0;
            }
            
            // delay for a while and take in button input
            // *doors open*
            printf("|--DOORS OPENED--|");
            for(i = 0; i < 150000; i++){
                // level input
                LVL_ONE = PORTK & 0x04;
                LVL_TWO = PORTK & 0x08;
                LVL_THREE = PORTK & 0x10;
                
                readBtnInput(); 
            }
            printf("|--DOORS ClOSED--|");
            
            //continue; // do not perform PR or lvlPartCnt manipulation
        }
    }
    while(1);
}
