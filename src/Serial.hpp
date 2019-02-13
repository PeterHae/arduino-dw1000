#pragma once
// TODO think aubout implementing...

enum NUMFORMAT{DEC, OCT, BIN, HEX};

class SerialPrint{
private:
public:
	static void print(const char*);
    //static void print(const char[]);
    static void print(char);
    static void print(unsigned char, int = DEC);
    static void print(int, int = DEC);
    static void print(unsigned int, int = DEC);
    static void print(long, int = DEC);
    static void print(unsigned long, int = DEC);
    static void print(double, int = 2);

	static void println(const char*);
    //static void println(const char[]);
    static void println(char);
    static void println(unsigned char, int = DEC);
    static void println(int, int = DEC);
    static void println(unsigned int, int = DEC);
    static void println(long, int = DEC);
    static void println(unsigned long, int = DEC);
    static void println(double, int = 2);
};

extern SerialPrint Serial;
