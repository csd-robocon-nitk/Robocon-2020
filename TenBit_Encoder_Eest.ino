#define BITS 10
#define LSB 
int pins_MSB_2_LSB[] = {53,51,41,39,37,35,49,47,45,43};
int arr[]={1,2,4,8,16,32,64,128,256,512};

void greytobinary(int *a, int count)
{
  int i;
  for(i = count - 2; i >= 0; i--)
  {
      a[i] = a[i]^a[i+1];
  }
}

int binarytodecimal(int *a, int count)
{
  int sum=0,i;
  for(i= 0;i < BITS;i++)
  sum = sum + (arr[i]*a[i]);
        return sum;
}

void setup(){
  int i;
  for (i = 0;i<BITS;i++){
    pinMode(pins_MSB_2_LSB[i], INPUT_PULLUP);
  }
  Serial.begin(9600);
  Serial.print("ready\n");
}

void loop(){
  int rawValues[BITS];
  int i, j;
  int result;

  j = BITS - 1;

  for (i = 0;i<BITS;i++){ 
    int readVal;
    readVal = !digitalRead(pins_MSB_2_LSB[i]);
    rawValues[j] = readVal;
    j--;
  }
  for(j = BITS -1; j >=0; j--){
    Serial.print(rawValues[j]); 
  }
  
  greytobinary(rawValues, BITS);
   Serial.print("\t");
  for(j = BITS-1; j>=0; j--){
    Serial.print(rawValues[j]); 
  }
   
  result = binarytodecimal(rawValues, BITS);

  Serial.print("\t");
  Serial.println(result);
  delay(100);
}
