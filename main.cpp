#include <iostream>
#include "encoder_draco.h"
#include "decoder_draco.h"


class InputParser {
public:
    InputParser(int& argc, char** argv) {
        for (int i = 1; i < argc; ++i)
            this->tokens.push_back(std::string(argv[i]));
    }
    /// @author iain
    const std::string& getCmdOption(const std::string& option) const {
        std::vector<std::string>::const_iterator itr;
        itr = std::find(this->tokens.begin(), this->tokens.end(), option);
        if (itr != this->tokens.end() && ++itr != this->tokens.end()) {
            return *itr;
        }
        static const std::string empty_string("");
        return empty_string;
    }
    /// @author iain
    bool cmdOptionExists(const std::string& option) const {
        return std::find(this->tokens.begin(), this->tokens.end(), option)
            != this->tokens.end();
    }
private:
    std::vector <std::string> tokens;
};

int main(int argc, char** argv) {
    InputParser input(argc, argv);
    EncoderDraco enc;
    DecoderDraco dec;
    // Command Line Arguments
    const std::string& input_file = input.getCmdOption("-i");
    if (input_file.empty()) {
        std::cout << "Please supply an input file with -i" << std::endl;
        return 0;
    }
    const std::string& output_file = input.getCmdOption("-o");

    uint8_t q_pos = 11;
    if (input.cmdOptionExists("-q")) {
       q_pos = std::stoi(input.getCmdOption("-q"));
    }
    uint8_t enc_speed = 10;
    if (input.cmdOptionExists("-e")) {
       enc_speed = std::stoi(input.getCmdOption("-e"));
    }
    uint8_t dec_speed = 10;
    if (input.cmdOptionExists("-d")) {
       dec_speed = std::stoi(input.getCmdOption("-d"));
    }

    bool use_pcl = false;
    if (input.cmdOptionExists("-p")) {
       use_pcl = true;
    }

    // End of Command Line Arguments

    if(use_pcl) {
        if(enc.EncodePcl(input_file, q_pos, enc_speed, dec_speed) == -1) {
            std::cout << "Encoding failed" << std::endl;
            return 0;
        }
    } else {
        if(enc.EncodePly(input_file, q_pos, enc_speed, dec_speed) == -1) {
            std::cout << "Encoding failed" << std::endl;
            return 0;
        }
    }
    if(dec.Decode(enc.GetBuffer().data(), enc.GetEncodedSize()) != 1) {
            std::cout << "Decoding failed" << std::endl;
            return 0;
    }
    if(use_pcl && output_file != "") {
        dec.SaveWithPCL(output_file);
    } else if(output_file != "") {
        dec.SaveWithDraco(output_file);
    }

    std::cout << "Complete" << std::endl;
    return 0;
}

