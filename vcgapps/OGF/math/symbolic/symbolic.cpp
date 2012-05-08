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

#include <OGF/math/symbolic/symbolic.h>
#include <OGF/basic/types/types.h>
#include <OGF/basic/debug/assert.h>

namespace OGF {

    namespace Symbolic {

        Variables x ;
        Parameters c ;

        static bool get_constant(const Node* n, double& value) {
            const Number* c = dynamic_cast<const Number*>(n) ;
            if(c == nil) { 
                return false ; 
            }
            value = c->value() ;
            return true ;
        }

        // Shoud be called for each argument of an operator
        // that was not used to construct the result.
        static void did_not_use(Node* n) {
            Counted::ref(n) ;
            Counted::unref(n) ;
        }

        //_________________________________________________________

        Expression::Expression() : Node_var(new Zero()) {
        }

        Expression::Expression(double x) : Node_var(new Number(x)) {
        }

        //_________________________________________________________

        Node_ptr::Node_ptr(double val) : ptr_(new Number(val)) { }

        Node_ptr::Node_ptr(int val) : ptr_(new Number(val)) { }

        bool Variable::is_constant() const {
            return false ;
        }

        double Variable::eval(const Context& context) const {
            ogf_parano_range_assert(index_, 0, int(context.variables.size() - 1)) ;
            return context.variables[index_] ;
        }
        
        void Variable::print(std::ostream& out) const {
            out << "x" << index_ ;
        }

        bool Variable::depends_on_variable(int var_index) const {
            return (var_index == index_) ;
        }

        Node* Variable::derivative(int var_index) const {
            return (var_index == index_) ? (Node*)(new One) : (Node*)(new Zero) ;
        }

        int Variable::max_variable_index() const {
            return index_ ;
        }

        bool Variable::depends_on_parameter(int prm_index) const {
            return false ;
        }
        
        int Variable::max_parameter_index() const {
            return -1 ;
        }

        bool Constant::is_constant() const {        
            return true ;
        }

        bool Constant::depends_on_variable(int var_index) const {
            return false ;
        }
        
        Node* Constant::derivative(int var_index) const {
            return new Zero ;
        }

        int Constant::max_variable_index() const {
            return -1 ;
        }
        
        double Number::eval(const Context&) const {
            return value_ ;
        }
        
        void Number::print(std::ostream& out) const {
            out << value_ ;
        }

        bool Number::depends_on_parameter(int prm_index) const {
            return false ;
        }
        
        int Number::max_parameter_index() const {
            return -1 ;
        }

        double Parameter::eval(const Context& context) const {
            ogf_parano_range_assert(index_, 0, int(context.parameters.size() - 1)) ;
            return context.parameters[index_] ;
        }
        
        void Parameter::print(std::ostream& out) const {
            out << "c" << index_ ;
        }

        bool Parameter::depends_on_parameter(int prm_index) const {
            return (prm_index == index_) ;
        }
        
        int Parameter::max_parameter_index() const {
            return index_ ;
        }

        //_________________________________________________________

        bool Operator::is_constant() const {
            return (left()->is_constant() && right()->is_constant()) ;
        }

        bool Operator::depends_on_variable(int var_index) const {
            return (left()->depends_on_variable(var_index) || right()->depends_on_variable(var_index)) ;
        }

        int Operator::max_variable_index() const {
            return ogf_max(left()->max_variable_index(), right()->max_variable_index()) ;
        }

        bool Operator::depends_on_parameter(int prm_index) const {
            return (left()->depends_on_parameter(prm_index) || right()->depends_on_parameter(prm_index)) ;
        }
        
        int Operator::max_parameter_index() const {
            return ogf_max(left()->max_parameter_index(), right()->max_parameter_index()) ;
        }

        double Plus::eval(const Context& context) const {
            return left()->eval(context) + right()->eval(context) ;
        }
        
        void Plus::print(std::ostream& out) const {
            out << "(" << left() << "+" << right() << ")" ;
        }

        Node* Plus::derivative(int i) const {
            return der(left(), i) + der(right(), i) ;
        }

        double Minus::eval(const Context& context) const {
            return left()->eval(context) - right()->eval(context) ;
        }
        
        void Minus::print(std::ostream& out) const {
            out << "(" << left() << "-" << right() << ")" ;
        }

        Node* Minus::derivative(int i) const {
            return der(left(), i) - der(right(), i) ;
        }

        double Times::eval(const Context& context) const {
            return left()->eval(context) * right()->eval(context) ;
        }
        
        void Times::print(std::ostream& out) const {
            out << "(" << left() << "*" << right() << ")" ;
        }

        Node* Times::derivative(int i) const {
            return der(left(), i) * right() + left() * der(right(), i) ;
        }

        double Divide::eval(const Context& context) const {
            double denom = right()->eval(context) ;
            ogf_parano_assert(denom != 0.0) ;
            return left()->eval(context) /  denom ;
        }
        
        void Divide::print(std::ostream& out) const {
            out << "(" << left() << "/" << right() << ")" ;
        }

        Node* Divide::derivative(int i) const {
            return (der(left(), i) * right() - left() * der(right(), i)) / (Node_ptr(left()) ^ 2) ;
        }


        Node_ptr operator+(Node_ptr left, Node_ptr right) {
            double left_val ;
            bool left_is_constant = get_constant(left, left_val) ;
            double right_val ;
            bool right_is_constant = get_constant(right, right_val) ;
            if(left_is_constant && right_is_constant) {
                did_not_use(left) ;
                did_not_use(right) ;
                return constant(left_val + right_val) ;
            }
            if(left_is_constant && left_val == 0.0) {
                did_not_use(left) ;
                return right ;
            }
            if(right_is_constant && right_val == 0.0) {
                did_not_use(right) ;
                return left ;
            }
            return new Plus(left, right) ;
        }
        
        Node_ptr operator-(Node_ptr left, Node_ptr right) {
            double left_val ;
            bool left_is_constant = get_constant(left, left_val) ;
            double right_val ;
            bool right_is_constant = get_constant(right, right_val) ;
            if(left_is_constant && right_is_constant) {
                did_not_use(left) ;
                did_not_use(right) ;
                return constant(left_val - right_val) ;
            }
            if(left_is_constant && left_val == 0.0) {
                did_not_use(left) ;
                return new Neg(right) ;
            }
            if(right_is_constant && right_val == 0.0) {
                did_not_use(right) ;
                return left ;
            }
            return new Minus(left, right) ;
        }
        
        Node_ptr operator*(Node_ptr left, Node_ptr right) {
            double left_val ;
            bool left_is_constant = get_constant(left, left_val) ;
            double right_val ;
            bool right_is_constant = get_constant(right, right_val) ;
            if(left_is_constant && right_is_constant) {
                did_not_use(left) ;
                did_not_use(right) ;
                return constant(left_val * right_val) ;
            }
            if(left_is_constant && left_val == 0.0) {
                did_not_use(left) ;
                did_not_use(right) ;
                return new Zero ;
            }
            if(left_is_constant && left_val == 1.0) {
                did_not_use(left) ;
                return right ;
            }
            if(right_is_constant && right_val == 0.0) {
                did_not_use(left) ;
                did_not_use(right) ;
                return new Zero ;
            }
            if(right_is_constant && right_val == 1.0) {
                did_not_use(right) ;
                return left ;
            }
            return new Times(left, right) ;
        }
        
        Node_ptr operator/(Node_ptr left, Node_ptr right) {
            double left_val ;
            bool left_is_constant = get_constant(left, left_val) ;
            double right_val ;
            bool right_is_constant = get_constant(right, right_val) ;

            if(right_is_constant && right_val == 0.0) {
                ogf_assert(false) ;
            }

            if(left_is_constant && right_is_constant) {
                did_not_use(left) ;
                did_not_use(right) ;
                return constant(left_val / right_val) ;
            }

            if(left_is_constant && left_val == 0.0) {
                did_not_use(left) ;
                did_not_use(right) ;
                return new Zero ;
            }

            if(left_is_constant && left_val == 1.0) {
                did_not_use(left) ;
                return new Pow(right,-1.0) ;
            }

            if(right_is_constant) {
                did_not_use(right) ;
                return new Times(constant(1.0 / right_val), left) ;
            }

            return new Divide(left, right) ;
        }
        
        Node_ptr operator^(Node_ptr left, Node_ptr right) {
            double left_val ;
            bool left_is_constant = get_constant(left, left_val) ;
            double right_val ;
            bool right_is_constant = get_constant(right, right_val) ;

            if(right_is_constant && right_val == 0.0 && left_is_constant && left_val == 0.0) {
                ogf_assert(false) ;
            }

            if(left_is_constant && right_is_constant) {
                did_not_use(left) ;
                did_not_use(right) ;
                return constant(pow(left_val,right_val)) ;
            }

            if(right_is_constant && right_val == 1.0) {
                did_not_use(right) ;
                return left ;
            }

            if(right_is_constant) {
                return new Pow(left, right_val) ;
            }

            return exp(right * ln(left)) ;
        }

        //_________________________________________________________

        bool Function::is_constant() const {
            return arg()->is_constant() ;
        }

        bool Function::depends_on_variable(int var_index) const {
            return arg()->depends_on_variable(var_index) ;
        }

        int Function::max_variable_index() const {
            return arg()->max_variable_index() ;
        }

        bool Function::depends_on_parameter(int prm_index) const {
            return arg()->depends_on_parameter(prm_index) ;
        }
        
        int Function::max_parameter_index() const {
            return arg()->max_parameter_index() ;
        }

        double Neg::eval(const Context& context) const {
            return -arg()->eval(context) ;
        }
        
        void Neg::print(std::ostream& out) const {
            out << "(-" << arg() << ")" ;
        }

        Node* Neg::derivative(int var_index) const {
            return - der(arg(), var_index) ;
        }

        double Sin::eval(const Context& context) const {
            return ::sin(arg()->eval(context)) ;
        }
        
        void Sin::print(std::ostream& out) const {
            out << "sin(" << arg() << ")" ;
        }

        Node* Sin::derivative(int var_index) const {
            return cos(arg()) * der(arg(), var_index) ;
        }

        double Cos::eval(const Context& context) const {
            return ::cos(arg()->eval(context)) ;
        }
        
        void Cos::print(std::ostream& out) const {
            out << "cos(" << arg() << ")" ;
        }

        Node* Cos::derivative(int var_index) const {
            return -sin(arg()) * der(arg(), var_index) ;
        }

        double Ln::eval(const Context& context) const {
            return ::log(arg()->eval(context)) ;
        }
        
        void Ln::print(std::ostream& out) const {
            out << "ln(" << arg() << ")" ;
        }

        Node* Ln::derivative(int var_index) const {
            return arg() / der(arg(), var_index) ;
        }

        double Exp::eval(const Context& context) const {
            return ::exp(arg()->eval(context)) ;
        }
        
        void Exp::print(std::ostream& out) const {
            out << "exp(" << arg() << ")" ;
        }

        Node* Exp::derivative(int var_index) const {
            return der(arg(),var_index) * exp(arg()) ;
        }


        double ArcSin::eval(const Context& context) const {
            return ::asin(arg()->eval(context)) ;
        }
        
        void ArcSin::print(std::ostream& out) const {
            out << "arcsin(" << arg() << ")" ;
        }

        Node* ArcSin::derivative(int var_index) const {
            return der(arg(),var_index) / sqrt( 1 - (arg()^2)) ;
        }
        
        double ArcCos::eval(const Context& context) const {
            return ::acos(arg()->eval(context)) ;
        }
        
        void ArcCos::print(std::ostream& out) const {
            out << "arccos(" << arg() << ")" ;
        }

        Node* ArcCos::derivative(int var_index) const {
            return - der(arg(),var_index) / sqrt( 1 - (arg()^2)) ;
        }

        double Pow::eval(const Context& context) const {
            return ::pow(arg()->eval(context),exponent()) ;
        }
        
        void Pow::print(std::ostream& out) const {
            out << "(" << arg() << "^" << exponent() << ")" ;
        }

        Node* Pow::derivative(int i) const {
            return exponent() * der(arg(), i) * (arg() ^ (exponent() - 1.0)) ;
        }

        Node_ptr operator-(Node_ptr arg) {
            double val ;
            if(get_constant(arg, val)) {
                did_not_use(arg) ;
                return constant(-val) ;
            }
            return new Neg(arg) ;
        }

        Node_ptr sin(Node_ptr arg) {
            double val ;
            if(get_constant(arg, val)) {
                did_not_use(arg) ;
                return constant(::sin(val)) ;
            }
            return new Sin(arg) ;
        }

        Node_ptr cos(Node_ptr arg) {
            double val ;
            if(get_constant(arg, val)) {
                did_not_use(arg) ;
                return constant(::cos(val)) ;
            }
            return new Cos(arg) ;
        }

        Node_ptr sqrt(Node_ptr arg) {
            double val ;
            if(get_constant(arg, val)) {
                did_not_use(arg) ;
                ogf_assert(val >= 0.0) ;
                return constant(::sqrt(val)) ;
            }
            return (arg ^ 0.5) ;
        }

        Node_ptr ln(Node_ptr arg) {
            double val ;
            if(get_constant(arg, val)) {
                did_not_use(arg) ;
                return constant(::log(val)) ;
            }
            return new Ln(arg) ;
        }

        Node_ptr exp(Node_ptr arg) {
            double val ;
            if(get_constant(arg, val)) {
                did_not_use(arg) ;
                return constant(::exp(val)) ;
            }
            return new Exp(arg) ;
        }

        Node_ptr arcsin(Node_ptr arg) {
            double val ;
            if(get_constant(arg, val)) {
                did_not_use(arg) ;
                return constant(::asin(val)) ;
            }
            return new ArcSin(arg) ;
        }

        Node_ptr arccos(Node_ptr arg) {
            double val ;
            if(get_constant(arg, val)) {
                did_not_use(arg) ;
                return constant(::acos(val)) ;
            }
            return new ArcCos(arg) ;
        }

        //_________________________________________________________

    }
}
