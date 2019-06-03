#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
//int test()
//{

//    RMatrix A(6,6);
//    RMatrix U(6), S(6,6), V(6), Q(6,6);
//    for(int i = 0; i < 6; i++)
//    {
//        for(int j = 0; j < 6; j++)
//            A.value[i][j] = i;
//    }
//    RMatrix::qrFullRMatrix(A,Q);

//    //    RMatrix::svdSim(A, U, S, V);
//    return 0;
//}
