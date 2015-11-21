class Coordinates {
    private:
        double x;
        double y;
        int order;

    public:
    Coordinates(double x, double y, double order) {
        this->x = x;
        this->y = y;
        this->order = order;
    }

    Coordinates() {
        this->x = 0;
        this->y = 0;
        this->order = -1;
    }

    double getX(){
        return x;
    }

    double getY(){
        return y;
    }

    int getOrder(){
        return order;
    }

    void setX(double x) {
        this->x = x;
    }

    void setY(double y) {
        this->y = y;
    }

    void setOrder(int order) {
        this->order = order;
    }

    Coordinates& operator=(Coordinates* rhs) {
        this->x = rhs->getX();
        this->y = rhs->getY();
        this->order = rhs->getOrder();
        return *(this);
    }


};