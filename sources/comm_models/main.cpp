#ifndef H_COMM_MODELS
#define H_COMM_MODELS
    #include "comm_models.hh"
#endif

using namespace std;

int main() {
    //needed for testing the features of CommModels Class
    int q = 40;
    CommModels averageFilter;
    averageFilter = CommModels(q, "sliding_window_filter");
    

    for (int i = 1; i <= q * 10; i++) {
        double call = (double) i;
        double repel = -2*call;
        double tStamp = call / q;
        averageFilter.update_comm_signals(call,repel,tStamp);

        //print values
        if(i % q == 0){
            std::cout<<i<<" "
                    <<"[call: "<<averageFilter.get_value("delta_call_signal")<<" = "
                    <<averageFilter.get_value("prev_call_signal")<<","
                    <<averageFilter.get_value("curr_call_signal")<<"],"
                    <<"[rep: "<<averageFilter.get_value("delta_repel_signal")<<" = "
                    <<averageFilter.get_value("prev_repel_signal")<<","
                    <<averageFilter.get_value("curr_repel_signal")<<"]"
                    <<std::endl;
        }
    }

    return 0;
}