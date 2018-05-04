#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
// set serialization needed by KeyFrame::mspChildrens ...
#include <boost/serialization/map.hpp>
// map serialization needed by KeyFrame::mConnectedKeyFrameWeights ...
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/base_object.hpp>
#include <fstream>
#include <iomanip>


int main(int argc, char * argv[]){

    std::size_t ntest ;
    std::string filename = "test.bin";

    if(argc<2) {
        std::ifstream in(filename, std::ios_base::binary);
        if (!in) {
            std::cerr << "Cannot Open : " << filename << std::endl;
            exit(-1);
        }
        std::cout << "Opening Mapfile: " << filename << std::flush;
        {
            boost::archive::binary_iarchive ia(in, boost::archive::no_header);
            ia >> ntest;
            std::cout << " ...done" << std::endl;
            std::cout << ntest << std::endl;
        }
        in.close();
    }
    else {
        std::ifstream in_text("test.txt");
        if (!in_text) {
            std::cerr << "Cannot Open : " << "test.txt" << std::endl;
            exit(-1);
        }
        std::cout << "Opening Mapfile: " << "test.txt" << std::flush;
        {
            boost::archive::text_iarchive ia_text(in_text);
            ia_text >> ntest;
            std::cout << " ...done" << std::endl;
            std::cout << ntest << std::endl;
        }
        in_text.close();
    }

    return 0;
};
