#include "BoostArchiver.h"
#include <fstream>
#include <iomanip>


int main(int argc, char * argv[]){

    std::size_t ntest = 454654546456456464324354345345;
    std::string filename = "test.bin";

    std::ofstream out(filename, std::ios_base::binary);
    if (!out)
    {
        std::cerr << "Cannot Write to Mapfile: " << filename << std::endl;
        exit(-1);
    }
    std::cout << "Saving Mapfile: " << filename << std::flush;
    {
        boost::archive::binary_oarchive oa(out, boost::archive::no_header);
        oa << ntest;
        std::cout << " ...done" << std::endl;
    }
    out.close();

    std::ofstream out_text("test.txt");
    if (!out_text)
    {
        std::cerr << "Cannot Write to Mapfile: " << filename << std::endl;
        exit(-1);
    }
    std::cout << "Saving Mapfile: " << "test.txt" << std::flush;
    {
        boost::archive::text_oarchive oa_text(out_text);
        oa_text << ntest;
        std::cout << " ...done" << std::endl;
    }
    out_text.close();


    return 0;
};
