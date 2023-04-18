int xStep = 2, xDir = 5;
int yStep = 3, yDir = 6;
int zStep = 4, zDir = 7;
int Enable = 8;
int xLim = 9, yLim = 10, zLim = 12;
bool xT = false, yT = false, zT = false;
bool empty = true;
int servo = 11, sState = 0;
int speeds = 1000;
double MINstep = 80;

String line;
char reading;
bool ended = false;
bool absolute = false;

double G, X = 0, Y = 0, Z = 0, F = 500, I, J, M, T;
double CX = 0, CY = 0, CZ = 1;

char *colors[] = {"FFA500", "FFA500", "FFA500", "FFA500", "FFA500", "FFA500", "FFA500", "FFA500", "FFA500", "FF0000", "0000FF", "000000"};

void Read(String GCC) {
  String GC = GCC + " ";
  char GCODE[GC.length()];
  //      Serial.println(GC);
  GC.toCharArray(GCODE, GC.length());
  char *name = NULL;
  name = strtok(GCODE, " ;\n");
  while (name != NULL)
  {
    String d = String(name);
    float tmp = d.substring(1, d.length()).toFloat();
    switch (name[0]) {
      case ('G'):
        G = tmp;
        break;
      case ('X'):
        X = tmp;
        break;
      case ('Y'):
        Y = tmp;
        break;
      case ('Z'):
        Z = tmp;
        break;
      case ('I'):
        I = tmp;
        break;
      case ('J'):
        J = tmp;
        break;
      case ('F'):
        F = tmp;
        break;
      case ('M'):
        M = tmp;
        G = -10;
        break;
      case ('T'):
        T = tmp;
      default:
        Serial.println(name[0]);
        Serial.println("IS not included");
        break;
    }

    name = strtok(NULL, " ");
  }
}

void Move(int dir, int Step, int DIRstate, int Stepcount) {
  digitalWrite(dir, DIRstate);
  for (int x = 0; x < Stepcount; x++) {
    digitalWrite(Step, HIGH);
    delayMicroseconds(speeds);
    digitalWrite(Step, LOW);
    delayMicroseconds(speeds);
  }
}

void Run(double xRun, double yRun, double zRun) {
  xRun = xRun * MINstep;
  yRun = yRun * MINstep;
  zRun = zRun * MINstep;

  double XYmax = (abs(xRun) > abs(yRun)) ? xRun : yRun;
  bool con = (abs(xRun) > abs(yRun)) ? true : false;
  double XYmin = (abs(xRun) < abs(yRun)) ? xRun : yRun;
  XYmin = round(abs(XYmax) / abs(XYmin));
  //  double MAXstep = (XY>zRun)?XY:zRun;

  for (int i = 0; i <= abs(XYmax); i++) {
    if (con) {
      Move(xDir, xStep, (xRun > 0) ? HIGH : LOW, 1);
      xRun -= (xRun > 0) ? 1 : -1;
    }
    else {
      Move(yDir, yStep, (yRun > 0) ? HIGH : LOW, 1);
      yRun -= (yRun > 0) ? 1 : -1;
    }
    if (i % (int)XYmin == 0) {
      if (!con) {
        Move(xDir, xStep, (xRun > 0) ? HIGH : LOW, 1);
        xRun -= (xRun > 0) ? 1 : -1;
      }
      else {
        Move(yDir, yStep, (yRun > 0) ? HIGH : LOW, 1);
        yRun -= (yRun > 0) ? 1 : -1;
      }
    }
  }

  Move(xDir, xStep, (xRun > 0) ? HIGH : LOW, abs(xRun));
  Move(yDir, yStep, (yRun > 0) ? HIGH : LOW, abs(yRun));
}

void Goabsolute(double x2, double y2) {
  Run(x2 - CX, y2 - CY, 0);
  CX = x2;
  CY = y2;
}

void Gorelative(double x2, double y2) {
  Goabsolute(CX + x2, CY + y2);
  X = 0;
  Y = 0;
}

void compile() {
  if (G == -10) {
    Serial.println("It's M");
  }
  else if (G == 90) {
    absolute = true;
  }
  else if (G == 0 || G == 1) {
    double netSpeed = F / 60; // mm per second
    speeds = 1000000 / (MINstep * netSpeed);
    speeds /= 2;
    if (absolute) {
      Goabsolute(X, Y);
    }
    else {
      Gorelative(X, Y);
    }
    if(Z == -1 && CZ==1){
      Move(zDir, zStep, HIGH, 500);
      CZ = 0;
    }
    else if(CZ==0){
      Move(zDir, zStep, LOW, 500);
      CZ = 1;
    }
  }
  else if (G == 2) {
    // Minimum Angle
    //    double OrX = CX + I, OrY = CY + J;
    //    double r = distance(OrX, OrY, CX, CY);
    //    double angle = (asin((MINstep / 2) / r) * 180 / 3.14) * 2;

    // Number of steps

  }
}

void Update() {
  xT = (digitalRead(xLim) == 0) ? false : true;
  yT = (digitalRead(yLim) == 0) ? false : true;
  zT = (digitalRead(zLim) == 0) ? false : true;
}

void AutoHome() {
  delay(200);
  while (!xT) {
    Move(xDir, xStep, LOW, 1);
    Update();
  }
  while (!yT) {
    Move(yDir, yStep, HIGH, 1);
    Update();
  }
  while (!zT) {
    Move(zDir, zStep, HIGH, 1);
    Update();
  }

  CX = 0;
  CY = 0;
  G = 0, X = 0, Y = 0, Z = 0;
  Move(zDir, zStep, LOW, 1000);
  CZ = 1;
}

void Pick(int p){
  // Homing Holder
  while (!zT) {
    Move(zDir, zStep, HIGH, 1);
    Update();
  }
  Move(zDir, zStep, LOW, 1000);
  Z = 0;
  delay(400);

  // Reaching the point of pen
  int reach = (abs(11-p)*23)+2;
  Goabsolute(reach, -367);
  if(empty){
    analogWrite(servo, 250);
  }
  Move(zDir, zStep, HIGH, 450);
  Goabsolute(CX, -382);
  Goabsolute(CX+2, -382);
  analogWrite(servo, 170);
  Move(zDir, zStep, HIGH, 550);

  Goabsolute(CX, -367);
  Move(zDir, zStep, LOW, 1000);
}
void place(int p){
  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(xStep, OUTPUT);
  pinMode(xDir, OUTPUT);
  pinMode(yStep, OUTPUT);
  pinMode(yDir, OUTPUT);
  pinMode(zStep, OUTPUT);
  pinMode(zDir, OUTPUT);
  pinMode(Enable, OUTPUT);
  pinMode(servo, OUTPUT);

  pinMode(xLim, INPUT);
  pinMode(yLim, INPUT);
  pinMode(zLim, INPUT);
  analogWrite(servo, 170); //170 - 250
  Serial.println("ok");
}

void loop() {
  // put your main code here, to run repeatedly:
  Update();
  if (Serial.available()) {
    reading = Serial.read();
    line += reading;
    if (reading == ';' || reading == ')' || reading == '\n') {
      Serial.println(line);
      if (line == "HH;") {
        AutoHome(); // 382
        Serial.println("Auto Homing Done");
      }
      else if (line[0] == '(') {
        if (line[1] == '<' && line[2] == 'G') {
          Serial.println("Need Tool Change");
          int g = line.indexOf("PenColor=") + 10;
          String col = "";
          for (int j = g; j < line.length(); j++) {
            if (line[j] != '"') {
              col += line[j];
            }
            else {
              break;
            }
          }
          for (int k = 0; k < 12; k++) {
            if ((String)colors[k] == (String)col) {
              Serial.println(k);
              Pick(k);
              break;
            }
          }
        }
      }
      else {
        Read(line);
        compile();
      }
      Serial.println(CY);
      Serial.println("ok");
      line = "";
    }
  }
}
