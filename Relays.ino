   void SetRelays(void)
 {
    if (bitRead(relay,0)) bitClear(PORTD, 5); //Digital Pin 5
    else bitSet(PORTD, 5); 
    if (bitRead(relay,1)) bitClear(PORTD, 6); //Digital Pin 6
    else bitSet(PORTD, 6); 
    if (bitRead(relay,2)) bitClear(PORTD, 7); //Digital Pin 7
    else bitSet(PORTD, 7); 
    if (bitRead(relay,3)) bitClear(PORTB, 0); //Digital Pin 8
    else bitSet(PORTB, 0); 
    if (bitRead(relay,4)) bitClear(PORTB, 1); //Digital Pin 9
    else bitSet(PORTB, 1); 
    if (bitRead(relay,5)) bitClear(PORTB, 2); //Digital Pin 10
    else bitSet(PORTB, 2); 
    if (bitRead(relay,6)) bitClear(PORTB, 3); //analog Pin 11
    else bitSet(PORTB, 3); 
    if (bitRead(relay,7)) bitClear(PORTB, 4); //Analog Pin 12
    else bitSet(PORTB, 4); 

    //fencerow code for my sprayers
    if(bitRead(fencerows, 0)) bitSet(PORTC, 3); //turn left fencerow on
    else bitSet(PORTC, 3);
    if(bitRead(fencerows, 1)) bitSet(PORTC, 4); //turn left fencerow on
    else bitSet(PORTC, 4);
  }
