#ifndef _POSE_ESTIMATION_EXCEPTIONS_HPP
#define _POSE_ESTIMATION_EXCEPTIONS_HPP

#include <stdexcept>

namespace pose_estimation
{

class WrongStateSizeException : public std::exception
{
public:
    explicit WrongStateSizeException(unsigned expected_state_size) :
        msg("The given state doesn't match the expected state size of " + expected_state_size) {}
    virtual char const * what() const throw() { return msg.c_str(); }
    ~WrongStateSizeException() throw() {}
protected:
    const std::string msg;
};

}

#endif