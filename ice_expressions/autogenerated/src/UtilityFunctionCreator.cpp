#include <iostream>
#include "UtilityFunctionCreator.h"

#include  "Plans/ProvideSupplies1484927081537.h"

using namespace std;
using namespace alicaAutogenerated;
namespace alica
{

    UtilityFunctionCreator::~UtilityFunctionCreator()
    {
    }

    UtilityFunctionCreator::UtilityFunctionCreator()
    {
    }

    shared_ptr<BasicUtilityFunction> UtilityFunctionCreator::createUtility(long utilityfunctionConfId)
    {
        switch (utilityfunctionConfId)
        {

            case 1484927081537:
                return make_shared<UtilityFunction1484927081537>();
                break;

            default:
                cerr << "UtilityFunctionCreator: Unknown utility requested: " << utilityfunctionConfId << endl;
                throw new exception();
                break;
        }
    }

}
