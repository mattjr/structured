/*
 *  OGF/Graphite: Geometry and Graphics Programming Library + Utilities
 *  Copyright (C) 2000 Bruno Levy
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *  If you modify this software, you should include a notice giving the
 *  name of the person performing the modification, the date of modification,
 *  and the reason for such modification.
 *
 *  Contact: Bruno Levy
 *
 *     levy@loria.fr
 *
 *     ISA Project
 *     LORIA, INRIA Lorraine, 
 *     Campus Scientifique, BP 239
 *     54506 VANDOEUVRE LES NANCY CEDEX 
 *     FRANCE
 *
 *  Note that the GNU General Public License does not permit incorporating
 *  the Software into proprietary programs. 
 */
 
#ifndef __OGF_MATH_SYMBOLIC_SYMBOLIC__
#define __OGF_MATH_SYMBOLIC_SYMBOLIC__

#include <OGF/math/common/common.h>
#include <OGF/basic/types/counted.h>
#include <OGF/basic/types/smart_pointer.h>

#include <iostream>
#include <vector>

// We got enough to try a first implementation of the generic symbolic / numerical
// solver. However, in the future, to get more performance, we should introduce
// n-ary operators with canonical ordering of the children:
//    Sum
//    Product 
// Check if we can find an implementation of an unification algorithm (and use
// a generic "Node" representation, with n children, and a "NodeType").
// We need to check the definition of the "normal semantic form" and use it.
// Evaluation: we can have a virtual machine (stack + array of function pointers)
//             or use SoftWire on Intel architectures
//                  a) use the system stack
//                  b) group the expressions with subtrees of depth <= 8, and use
//                      the registers of the math processor
//                  c) check what we can do with the multi-media registers, katmai
//                      instructions etc...
// It would be cool to have also a parser: Sym::Expression = Sym::parse("x1*x2/cos(x3)")

namespace OGF {

    namespace Symbolic {

        struct Context {
            std::vector<double> variables ;
            std::vector<double> parameters ;
        } ;
 
        class Node ;

        /**
         * Node_ptr encapsulates a pointer to a Node,
         * we need it to allow operators overloading
         * (operators overloading do not work with Node*,
         * it needs a class).
         */
        class MATH_API Node_ptr {
        public:
            Node_ptr() : ptr_(nil) { }
            Node_ptr(const Node_ptr& rhs) : ptr_(rhs.ptr_) { }
            Node_ptr(Node* ptr) : ptr_(ptr) { }
            Node_ptr(double val) ;
            Node_ptr(int val) ;

            Node_ptr& operator=(const Node_ptr& rhs) { ptr_ = rhs.ptr_ ; return *this ; }
            Node_ptr& operator=(Node* rhs) { ptr_ = rhs ; return *this ; }
 
            operator Node*() const { return ptr_ ; }
            Node* operator->()      { return ptr_ ; }
            Node& operator*() const { return *ptr_ ; }
        private:
            Node* ptr_ ;
        } ;
        
        /**
         * Node is an element of a mathematical expression.
         */
        class MATH_API Node : public Counted {
        public:
            virtual double eval(const Context& context) const = 0 ;
            virtual void print(std::ostream& out) const = 0 ;
            virtual bool is_constant() const = 0 ; 
            virtual bool depends_on_variable(int var_index) const = 0 ;
            virtual int max_variable_index() const = 0 ;
            virtual bool depends_on_parameter(int prm_index) const = 0 ;
            virtual int max_parameter_index() const = 0 ;
            virtual Node* derivative(int var_index) const = 0 ; 
        } ;

        typedef SmartPointer<Node> Node_var ;

        /**
         * Expression is the user-type for mathematical
         * expression. Expression encapsulates a Node 
         * SmartPointer.
         */
        class MATH_API Expression : public Node_var {
        public:
            Expression(Node_ptr ptr) : Node_var(ptr) { }
            Expression(Node* ptr) : Node_var(ptr) { }
            Expression() ;
            Expression(double x) ;
            operator Node_ptr() const {
	       const Node_var& n = *this ;
	       return (Node*)(n) ;
//	       return Node_var::operator Node*() ; 
	    }
            Expression& operator=(const Expression& rhs) {
                Node_var::operator=(rhs) ;
                return *this ;
            }
            Expression& operator=(Node_ptr rhs) {
                Node_var::operator=(rhs) ;
                return *this ;
            }
        } ;

        class MATH_API Variable : public Node {
        public:
            Variable(int index) : index_(index) { }
            int index() const { return index_ ; }
            virtual double eval(const Context& context) const ;
            virtual void print(std::ostream& out) const ;
            virtual bool is_constant() const ; 
            virtual bool depends_on_variable(int var_index) const ;
            virtual int max_variable_index() const ;
            virtual bool depends_on_parameter(int prm_index) const ;
            virtual int max_parameter_index() const ;
            virtual Node* derivative(int var_index) const ; 
        private:
            int index_ ;
        } ;

        // Cannot declare it inline, else the linker barks at x, 
        // as a multiply defined symbol.
        class MATH_API Variables {
        public:
            Node_ptr operator[](int index) { return new Variable(index) ; }
        } ;
        extern MATH_API Variables x ;

        class MATH_API Constant : public Node {
        public:
            virtual bool is_constant() const ; 
            virtual bool depends_on_variable(int var_index) const ;
            virtual Node* derivative(int var_index) const ; 
            virtual int max_variable_index() const ;
        } ;

        class MATH_API Number : public Constant {
        public:
            Number(double value) : value_(value) { }
            double value() const { return value_ ; }
            virtual double eval(const Context& context) const ;
            virtual void print(std::ostream& out) const ;
            virtual bool depends_on_parameter(int prm_index) const ;
            virtual int max_parameter_index() const ;
        private:
            double value_ ;
        } ;

        class MATH_API Zero : public Number {
        public:
            Zero() : Number(0.0) { }
        } ;

        class MATH_API One : public Number {
        public:
            One() : Number(1.0) { }
        } ;

        class MATH_API Parameter : public Constant {
        public:
            Parameter(int index) : index_(index) { }
            int index() const { return index_ ; }
            virtual double eval(const Context& context) const ;
            virtual void print(std::ostream& out) const ;
            virtual bool depends_on_parameter(int prm_index) const ;
            virtual int max_parameter_index() const ;
        private:
            int index_ ;
        } ;

        // Cannot declare it inline, else the linker barks at x, 
        // as a multiply defined symbol.
        class MATH_API Parameters {
        public:
            Node_ptr operator[](int index) { return new Parameter(index) ; }
        } ;
        extern MATH_API Parameters c ;


        inline Node_ptr constant(double value) {
            if(value == 0.0) { return new Zero ; }
            if(value == 1.0) { return new One ;  }
            return new Number(value) ;
        }

        inline Node_ptr der(Node* e, int var_index) {
            return e->derivative(var_index) ;
        }

        //__________________________________________________________________

        class MATH_API Operator : public Node {
        public:
            Operator(Node* left, Node* right) : left_(left), right_(right) {
            }
            Node_ptr left() const { return (Node*)left_ ; }
            Node_ptr right() const { return (Node*)right_ ; }
            virtual bool is_constant() const ; 
            virtual bool depends_on_variable(int var_index) const ;
            virtual int max_variable_index() const ;
            virtual bool depends_on_parameter(int prm_index) const ;
            virtual int max_parameter_index() const ;
        private:
            Node_var left_ ;
            Node_var right_ ;
        } ;

        class MATH_API Plus : public Operator {
        public:
            Plus(Node* left, Node* right) : Operator(left, right) {}
            virtual double eval(const Context& context) const ;
            virtual void print(std::ostream& out) const ;
            virtual Node* derivative(int var_index) const ;
        } ;

        class MATH_API Minus : public Operator {
        public:
            Minus(Node* left, Node* right) : Operator(left, right) {}
            virtual double eval(const Context& context) const ;
            virtual void print(std::ostream& out) const ;
            virtual Node* derivative(int var_index) const ;
        } ;

        class MATH_API Times : public Operator {
        public:
            Times(Node* left, Node* right) : Operator(left, right) {}
            virtual double eval(const Context& context) const ;
            virtual void print(std::ostream& out) const ;
            virtual Node* derivative(int var_index) const ;
        } ;

        class MATH_API Divide : public Operator {
        public:
            Divide(Node* left, Node* right) : Operator(left, right) {}
            virtual double eval(const Context& context) const ;
            virtual void print(std::ostream& out) const ;
            virtual Node* derivative(int var_index) const ;
        } ;


        /*
         * Note: it seems that the C++ hat operator does not have the same
         * priority as a power. When in doubt, use parentheses.
         */

        Node_ptr MATH_API operator+( Node_ptr left, Node_ptr right ) ;
        Node_ptr MATH_API operator-( Node_ptr left, Node_ptr right ) ;
        Node_ptr MATH_API operator*( Node_ptr left, Node_ptr right ) ;
        Node_ptr MATH_API operator/( Node_ptr left, Node_ptr right ) ;
        Node_ptr MATH_API operator^( Node_ptr left, Node_ptr right ) ;

        //__________________________________________________________________

        class MATH_API Function : public Node {
        public:
            Function(Node* arg) : arg_(arg) { }
            Node_ptr arg() const { return (Node*)arg_ ; }
            virtual bool is_constant() const ; 
            virtual bool depends_on_variable(int var_index) const ;
            virtual int max_variable_index() const ;
            virtual bool depends_on_parameter(int prm_index) const ;
            virtual int max_parameter_index() const ;
        private:
            Node_var arg_ ;
        } ;

        class MATH_API Neg : public Function {
        public:
            Neg(Node* arg) : Function(arg) { }
            virtual double eval(const Context& context) const ;
            virtual void print(std::ostream& out) const ;
            virtual Node* derivative(int var_index) const ;
        } ;

        class MATH_API Sin : public Function {
        public:
            Sin(Node* arg) : Function(arg) { } 
            virtual double eval(const Context& context) const ;
            virtual void print(std::ostream& out) const ;
            virtual Node* derivative(int var_index) const ;
        } ;

        class MATH_API Cos : public Function {
        public:
            Cos(Node* arg) : Function(arg) { }
            virtual double eval(const Context& context) const ;
            virtual void print(std::ostream& out) const ;
            virtual Node* derivative(int var_index) const ;
        } ;

        class MATH_API Ln : public Function {
        public:
            Ln(Node* arg) : Function(arg) { }
            virtual double eval(const Context& context) const ;
            virtual void print(std::ostream& out) const ;
            virtual Node* derivative(int var_index) const ;
        } ;

        class MATH_API Exp : public Function {
        public:
            Exp(Node* arg) : Function(arg) { }
            virtual double eval(const Context& context) const ;
            virtual void print(std::ostream& out) const ;
            virtual Node* derivative(int var_index) const ;
        } ;

        class MATH_API Pow : public Function {
        public:
            Pow(Node* arg, double exponent) : Function(arg), exponent_(exponent) {}
            double exponent() const { return exponent_ ; }
            virtual double eval(const Context& context) const ;
            virtual void print(std::ostream& out) const ;
            virtual Node* derivative(int var_index) const ;
        private:
            double exponent_ ;
        } ;

        class MATH_API ArcSin : public Function {
        public:
            ArcSin(Node* arg) : Function(arg) { }
            virtual double eval(const Context& context) const ;
            virtual void print(std::ostream& out) const ;
            virtual Node* derivative(int var_index) const ;

        } ;

        class MATH_API ArcCos : public Function {
        public:
            ArcCos(Node* arg) : Function(arg) { }
            virtual double eval(const Context& context) const ;
            virtual void print(std::ostream& out) const ;
            virtual Node* derivative(int var_index) const ;
        } ;


        Node_ptr MATH_API operator-(Node_ptr arg) ;
        Node_ptr MATH_API sin(Node_ptr arg) ;
        Node_ptr MATH_API cos(Node_ptr arg) ;
        Node_ptr MATH_API sqrt(Node_ptr arg) ;
        Node_ptr MATH_API ln(Node_ptr arg) ;
        Node_ptr MATH_API exp(Node_ptr arg) ;
        Node_ptr MATH_API arcsin(Node_ptr arg) ;
        Node_ptr MATH_API arccos(Node_ptr arg) ;

        //__________________________________________________________________

    }

    inline std::ostream& operator<<(std::ostream& out, Symbolic::Node_ptr n) {
        n->print(out) ;
        return out ;
    }

    inline std::ostream& operator<<(std::ostream& out, Symbolic::Expression n) {
        n->print(out) ;
        return out ;
    }

}

#endif
