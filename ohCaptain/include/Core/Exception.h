
//
// Created by peer23peer on 7/17/16.
//

#pragma once

#include <iostream>
#include <exception>
#include <string>

namespace oCpt {
    class oCptException : public std::exception {
    public:
        oCptException(std::string msg="exception!", int id=-1) : _msg(msg), _id(id) {}
        ~oCptException() throw() {}
        const char* what() const throw() { return _msg.c_str(); }
    private:
        std::string _msg;
        int _id;
    };
}