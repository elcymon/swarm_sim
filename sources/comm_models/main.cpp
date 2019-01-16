#include "comm_models.hh"

using namespace std;

int main() {
    //needed for testing the features of CommModels Class
    CommModels averageFilter = CommModels(40, "average_filter");
    

    for (int i = 1; i <= 120; i++) {
        double call = (double) i;
        double repel = call * 2;
        double tStamp = call / 40;
        averageFilter.update_comm_signals(i,2*i,i/40);

        //print values
        std::cout<<i<<" "
                    <<"["<<averageFilter.get_value("prev_call_signal")<<","
                    <<averageFilter.get_value("curr_call_signal")<<"],"
                    <<"["<<averageFilter.get_value("prev_repel_signal")<<","
                    <<averageFilter.get_value("curr_repel_signal")<<"]"
                    <<std::endl;
    }

    return 0;
}