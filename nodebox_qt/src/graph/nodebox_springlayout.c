#include "Python.h"
#include "arrayobject.h"
#include <math.h>
#include "nodebox_springlayout.h"

static PyMethodDef _nodebox_springlayoutMethods[] = {
	{"spring_force", spring_force, METH_VARARGS},
	{NULL, NULL}     /* Sentinel - marks the end of this structure */
};

void initnodebox_springlayout()  {
	(void) Py_InitModule("nodebox_springlayout", _nodebox_springlayoutMethods);
	import_array();  // Must be present for NumPy.  Called first after above line.
    srand(time(NULL));
}

double **ptrvector(long n)  
{
    double **v;
    v = (double **) malloc((size_t) (n*sizeof(double)));
    if (!v)   
    {
        printf("In **ptrvector. Allocation of memory for double array failed.");
        exit(0);  
    }
    return v;
}

double **pymatrix_to_Carrayptrs(PyArrayObject *arrayin)  
{
    int i, n, m;
    double **c, *a;
    n = arrayin->dimensions[0];
    m = arrayin->dimensions[1];
    c = ptrvector(m);
    a = (double *) arrayin->data;  /* pointer to arrayin data as double */

    for (i = 0; i < m; i++)
    {
        c[i] = a + i*n;
    }
    return c;
}

void free_Carrayptrs(double **v)  
{
      free((char*) v);
}

//Takes as input 2xn np.matrix
//returns 2xn np.matrix of forces for each node
static PyObject *spring_force(PyObject *self, PyObject *args)
{
	PyArrayObject *matin, *matout, *elen;
    double r, k, r0, r_cur;
    //double **cin, **cout, **elenp;
    int dims[2];
    npy_intp i, j;

	if (!PyArg_ParseTuple(args, "O!O!ddd", &PyArray_Type, &matin, &PyArray_Type, &elen, &k, &r, &r0))  
        return NULL;
	if (NULL == matin)  
        return NULL;

	dims[0] = matin->dimensions[0];
	dims[1] = matin->dimensions[1];
	matout = (PyArrayObject *) PyArray_FromDims(2, dims, NPY_DOUBLE);

	//cin = pymatrix_to_Carrayptrs(matin);
	//cout = pymatrix_to_Carrayptrs(matout);
	//elenp = pymatrix_to_Carrayptrs(elen);

    //zero out
    for (i = 0; i < dims[0]; i++)
    {
        for (j = 0; j < dims[1]; j++)
        {
            *((double *) PyArray_GETPTR2(matout, i, j)) = 0.;
        }
    }

    //printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx@@@\n");
    //printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
    //printf("dim0 %d dim1 %d\n", dims[0], dims[1]);
    //for (i = 0; i < dims[0]; i++)
    //{
    //    for (j = 0; j < dims[1]; j++)
    //    {
    //        printf("%f ", *((double *) PyArray_GETPTR2(matin, i, j)));
    //    }
    //    printf("\n");
    //}

    //printf("elenp\n");
    //for (i = 0; i < elen->dimensions[0]; i++)
    //{
    //    for (j = 0; j < elen->dimensions[1]; j++)
    //    {
    //        printf("%f ", *((double *) PyArray_GETPTR2(elen, i, j)));
    //    }
    //    printf("\n");
    //}

    for (i = 0; i < dims[1]; i++)
    {
        double xi, yi;
        //xi = cin[i][0];
        //yi = cin[i][1];
        xi = *((double *) PyArray_GETPTR2(matin, 0, i));
        yi = *((double *) PyArray_GETPTR2(matin, 1, i));

        for (j = i+1; j < dims[1]; j++)
        {
            //printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
            //printf("i %d j %d\n", (int) i, (int) j);
            //calculate distance
            double xj, yj, d2, dx, dy, f, d;
            //xj = cin[j][0];
            //yj = cin[j][1];
            xj = *((double *) PyArray_GETPTR2(matin, 0, j));
            yj = *((double *) PyArray_GETPTR2(matin, 1, j));

            dx = xj - xi;
            dy = yj - yi;
            d2 = pow(dx, 2.) + pow(dy, 2.);
            //if (i != j)
            //{
            if (d2 < .01)
            {
                //printf("@ d2 %f\n", d2);
                dx = ((float) rand() / (float) RAND_MAX) * .1 + .1;
                dy = ((float) rand() / (float) RAND_MAX) * .1 + .1;
                d2 = pow(dx, 2.) + pow(dy, 2.);
                //printf("@ d2 %f dx %f dy %f\n", d2, dx, dy);
            }
            //}
            d = pow(d2, .5);
            //printf("d2 %f d1 %f\n", d2, d);
            //
            if ((*((double *) PyArray_GETPTR2(elen, 0, j)) < 1) ||
               (*((double *) PyArray_GETPTR2(elen, 0, i)) < 1))
                r_cur = r0; 
            else
                r_cur = r;


            //if ((elenp[j][0] < 1) || (elenp[i][0] < 1))
            //    r_cur = r0; 
            //else
            //    r_cur = r;

            //calculate repulsion
            if (d < r_cur)
            {
                f = pow(k, 2.) / d2;

                //cout[j][0] = cout[j][0] + (f * dx);
                //cout[j][1] = cout[j][1] + (f * dy);
                //                       
                //cout[i][0] = cout[i][0] - (f * dx);
                //cout[i][1] = cout[i][1] - (f * dy);

                *((double *) PyArray_GETPTR2(matout, 0, j)) = *((double *) PyArray_GETPTR2(matout, 0, j)) + (f * dx);
                *((double *) PyArray_GETPTR2(matout, 1, j)) = *((double *) PyArray_GETPTR2(matout, 1, j)) + (f * dy);
                                                                                                        
                *((double *) PyArray_GETPTR2(matout, 0, i)) = *((double *) PyArray_GETPTR2(matout, 0, i)) - (f * dx);
                *((double *) PyArray_GETPTR2(matout, 1, i)) = *((double *) PyArray_GETPTR2(matout, 1, i)) - (f * dy);


                //printf("f %f dx %f dy %f\n", f, dx, dy);
            }
        }
    }

    //free_Carrayptrs(cin);
	//free_Carrayptrs(cout);
	return PyArray_Return(matout);
}    




//double **pymatrix_to_Carrayptrs(PyArrayObject *arrayin)  
//{
//    double **c, *a;
//    int i,n,m;
//    
//    n = arrayin->dimensions[0];
//    m = arrayin->dimensions[1];
//    c = ptrvector(n);
//    a = (double *) arrayin->data;  /* pointer to arrayin data as double */
//    for (i = 0; i < n; i++)  
//    {
//        c[i] = a + i*m;  
//    }
//    return c;
//}




