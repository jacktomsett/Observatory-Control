#ifndef _DATE_
#define _DATE_

#include <string>
#include <iostream>

class Date
{
    private:
        int year;
        int month;
        int day;

    public:
        //Getters...
        int getyear();
        int getmonth();
        int getday();

        //Functions for printing. To make this class general purpose they will just return a string, since I don't know what program output will look like.
        //Second function will take a format specifier
        std::string print();
        std::string print(std::string);

        Date();
        Date(int,int,int);

        //Operators
        Date operator++(int);
        bool operator==(Date);
        bool operator!=(Date);
        bool operator<(Date);
        bool operator>(Date);
        bool operator>=(Date);
        bool operator<=(Date);

};

#endif // _DATE_
