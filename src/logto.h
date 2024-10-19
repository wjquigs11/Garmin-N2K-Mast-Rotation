
#ifndef LOGTO_H
#define LOGTO_H

class logTo {
public:
    static bool logToSerial;
    static void logToAll(String s);
    //logTo() {}
    static const int ASIZE = 21;
    static String commandList[ASIZE];
};
#endif