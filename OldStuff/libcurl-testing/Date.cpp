#ifndef _DATESRC_
#define _DATESRC_
#include <Date.h>


//==================Constructors====================//

//Create an empty date... maybe later initialise it with system date
Date::Date()
{
    //Nothing to do here...
};

//Create date based on provided values
Date::Date(int y, int m, int d)
{
    //Check dat and month are correct
    try
    {
        if( m>12 || d>31)
        {
            throw 505;
        }
        else if ( d>30 && (m==4 || m==6 || m==9 || m==11))
        {
            throw 505;
        }
        else if (d>28 && m==2)
        {
            throw 505;
        }
    }
    catch (...)
    {
        std::cout << "ERROR: Invalid date detected" << std::endl;
    }

    //Initialize class:
    year = y;
    month = m;
    day = d;
};

//==================Operators====================//
//Increment date by one day
Date Date::operator++(int)
{
    day = day + 1;
    if (day > 31)
    {
        day = 1;
        month = month + 1;
    }
    else if (day > 30 && (month == 4 || month == 6 || month == 9 || month == 11) )
    {
        day = 1;
        month = month + 1;
    }
    else if (day > 28 && month == 2)
    {
        day = 1;
        month = month + 1;
    }

    if (month = 13)
    {
        month = 1;
        year = year + 1;
    }
    return Date(year,month,day);
};

//Check is two dates are equal
bool Date::operator==(Date b)
{
    if ( this->getday() == b.getday() && this->getmonth() == b.getmonth() && this->getyear() == b.getyear() )
    {
        return true;
    }

    else
    {
        return false;
    }
};

//Chck if two dates are not equal
bool Date::operator!=(Date b)
{
    if ( this->getday() == b.getday() && this->getmonth() == b.getmonth() && this->getyear() == b.getyear() )
    {
        return false;
    }

    else
    {
        return true;
    }
};

//Check if one date is less than another. In this context being less than
//means earlier.
bool Date::operator<(Date b)
{
    if (this->getyear() < b.getyear())
    {
        return true;
    }
    else if (this->getyear() == b.getyear() && this->getmonth() < b.getmonth())
    {
        return true;
    }
    else if (this->getyear() == b.getyear() && this->getmonth() == b.getmonth() && this->getday() < b.getday())
    {
        return true;
    }
    else
    {
        return false;
    }
};

//Check is one date is less than or equal to another. In this context being less than
//means earler
bool Date::operator<=(Date b)
{
    if (this->getyear() < b.getyear())
    {
        return true;
    }
    else if (this->getyear() == b.getyear() && this->getmonth() < b.getmonth())
    {
        return true;
    }
    else if (this->getyear() == b.getyear() && this->getmonth() == b.getmonth() && this->getday() < b.getday())
    {
        return true;
    }
    else if (this->getyear() == b.getyear() && this->getmonth() == b.getmonth() && this->getday() == b.getday())
    {
        return true;
    }
    else
    {
        return false;
    }
};

//Check if one date is greater than another. In this context being
//greater than means later.
bool Date::operator>(Date b)
{
    if (this->getyear() > b.getyear())
    {
        return true;
    }
    else if (this->getyear() == b.getyear() && this->getmonth() > b.getmonth())
    {
        return true;
    }
    else if (this->getyear() == b.getyear() && this->getmonth() == b.getmonth() && this->getday() > b.getday())
    {
        return true;
    }
    else
    {
        return false;
    }
};

//Check if one date is greater than or equal to another. In this context being
//greater than means later.
bool Date::operator>=(Date b)
{
    if (this->getyear() > b.getyear())
    {
        return true;
    }
    else if (this->getyear() == b.getyear() && this->getmonth() > b.getmonth())
    {
        return true;
    }
    else if (this->getyear() == b.getyear() && this->getmonth() == b.getmonth() && this->getday() > b.getday())
    {
        return true;
    }
    else if (this->getyear() == b.getyear() && this->getmonth() == b.getmonth() && this->getday() == b.getday())
    {
        return true;
    }
    else
    {
        return false;
    }
};

//===============Public member functions===========//
//Return the year, month or day
int Date::getyear()
{
    return year;
};

int Date::getmonth()
{
    return month;
};

int Date::getday()
{
    return day;
};

//Return a string containing the date
std::string Date::print()
{
    std::string yearstring = std::to_string(year);
    std::string monthstring = std::to_string(month);
    std::string daystring = std::to_string(day);

    if (daystring.length() == 1)
    {
        daystring = "0" + daystring;
    }
    if (monthstring.length() == 1)
    {
        monthstring = "0" + monthstring;
    }

    return yearstring + "-" + monthstring + "-" + daystring;   
};

#endif // _DATESRC_
