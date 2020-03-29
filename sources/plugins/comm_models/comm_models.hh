#ifndef H_COMMHEADERS
#define H_COMMHEADERS
    #include <iostream>
    #include <string>
    #include <deque>
    #include <numeric>
    #include <iterator>
#endif

class CommModels {
    private:
        double prev_call_signal;
        double curr_call_signal;
        std::deque<double> call_queue; //twice size of queue_size to hold values to compute prev & curr communicated value
        
        double prev_repel_signal;
        double curr_repel_signal;
        std::deque<double> repel_queue;

        double queue_size; //size of array to compute single value (e.g. prev_call_signal)

        std::deque<double> time_stamp; //time stamp on the communicated signals

        // measure of change in intensity between prev and curr communication signals
        double delta_call_signal;
        double delta_repel_signal;

        std::deque<double> attraction_x, attraction_y, repulsion_x, repulsion_y;
        std::map<std::string,double> resultant_vectors;
        /*
        Declare methods to handing different filtering approaches as private
        they should be used to update desired values based on user's choice
        */
        std::string filter_type;
        /*
        Average filter works on non-overlapping chunk of data to compute a single value
        */
        void average_filter(std::deque<double> *signal, double *prev, double *curr);

        /*
        Sliding window works on sliding twice the length of the queue size
        window is then divided into two. one for the prev and second for the curr
        signal value
        */
       void sliding_window_filter(std::deque<double> *signal, double *prev, double *curr);

        /*
        Linear regression equation fits a regression line to data that is twice the queue_size
        prev and curr values are updated as 0 and 1, which is dependent on whether the gradient
        is positive or negative
        */
       void linear_regression_eqn(std::deque<double> *signal,std::deque<double> *tStamp,
                                    double *prev, double *curr);
        //general filter implementation (used for including vector based communication noise filtering)
        double filter(std::deque<double> *signal, std::string *filter_type);
        
    public:
        CommModels();
        CommModels(int qSize, std::string filterType);

        double get_value(std::string desired_value);//used to access signals and gradients
        
        //accept communicated information and time stamp
        void update_comm_signals(double call_comm, double repel_comm, double t, double att_x, double att_y, double rep_x, double rep_y);

        double get_vector_component(std::string name);


};
