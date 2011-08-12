#include "Python.h"
#include "arrayobject.h"

static PyObject *spring_force(PyObject *self, PyObject *args);
double **ptrvector(long n);
double **pymatrix_to_Carrayptrs(PyArrayObject *arrayin);
void free_Carrayptrs(double **v);
static PyObject *spring_force(PyObject *self, PyObject *args);
void initnodebox_springlayout(void);
