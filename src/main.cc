#include <include/optimizer.h>

int main(int argc, char **argv)
{
    CeresOptimizer optimizer;
    optimizer.AddResidual();
    optimizer.SolveProblem();

    return 0;
}