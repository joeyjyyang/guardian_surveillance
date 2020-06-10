/*
* This example demonstrates sending a SMS or MMS in C++ using the Twilio REST
* APIs.  After building, you should be able to run it with:
* 
* ./bin/cpp_demo
* 
*/

#include <iostream>
#include <string>
#include <memory>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "twilio.hh"


int main(int argc, char * argv[])
{
        std::string account_sid = "AC33a76c0fcf93b551f413f7bce2096b02";
        std::string auth_token = "2b8cb36fbd7ae06ddb62c4a726ad1708";
        std::string message = "testing";
        std::string from_number = "+16473720640";
        std::string to_number = "+16479663926";

        // Instantiate a twilio object and call send_message
        std::string response;
        auto twilio = std::make_shared<twilio::Twilio>(
            account_sid, 
            auth_token
        );
        
        bool message_success = twilio->send_message(
                to_number, 
                from_number, 
                message
        );

        // Report success or failure
        if (!message_success) {
                std::cout << "Message send failed." << std::endl;
                return -1;
	}
        else 
	{
                std::cout << "SMS sent successfully!" << std::endl;
        }

        return 0;
}
