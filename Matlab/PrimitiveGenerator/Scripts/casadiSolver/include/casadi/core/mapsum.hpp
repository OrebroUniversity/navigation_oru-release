/*
 *    This file is part of CasADi.
 *
 *    CasADi -- A symbolic framework for dynamic optimization.
 *    Copyright (C) 2010-2014 Joel Andersson, Joris Gillis, Moritz Diehl,
 *                            K.U. Leuven. All rights reserved.
 *    Copyright (C) 2011-2014 Greg Horn
 *
 *    CasADi is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    CasADi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with CasADi; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


#ifndef CASADI_MAPSUM_HPP
#define CASADI_MAPSUM_HPP

#include "function_internal.hpp"

/// \cond INTERNAL

namespace casadi {

  /** Map with reduce_in/reduce_out
      \author Joris Gillis

      \date 2019
  */
  class CASADI_EXPORT MapSum : public FunctionInternal {
  public:
    // Create function (use instead of constructor)
    static Function create(const std::string& name,
                           const std::string& parallelization,
                           const Function& f, casadi_int n,
                           const std::vector<bool>& reduce_in,
                           const std::vector<bool>& reduce_out,
                           const Dict& opts=Dict());

    // Create function (use instead of constructor)
    static Function create(const std::string& name,
                           const std::string& parallelization,
                           const Function& f, casadi_int n,
                           const std::vector<casadi_int>& reduce_in,
                           const std::vector<casadi_int>& reduce_out,
                           const Dict& opts=Dict());

    /** \brief Destructor */
    ~MapSum() override;

    /** \brief Get type name */
    std::string class_name() const override {return "MapSum";}

    /// @{
    /** \brief Sparsities of function inputs and outputs */
    Sparsity get_sparsity_in(casadi_int i) override {
      return repmat(f_.sparsity_in(i), 1, reduce_in_[i] ? 1 : n_);
    }
    Sparsity get_sparsity_out(casadi_int i) override {
      return repmat(f_.sparsity_out(i), 1, reduce_out_[i] ? 1 : n_);
    }
    /// @}

    /** \brief Get default input value */
    double get_default_in(casadi_int ind) const override { return f_.default_in(ind);}

    ///@{
    /** \brief Number of function inputs and outputs */
    size_t get_n_in() override { return f_.n_in();}
    size_t get_n_out() override { return f_.n_out();}
    ///@}

    ///@{
    /** \brief Names of function input and outputs */
    std::string get_name_in(casadi_int i) override { return f_.name_in(i);}
    std::string get_name_out(casadi_int i) override { return f_.name_out(i);}
    /// @}

    /** \brief  Evaluate or propagate sparsities */
    template<typename T>
    int eval_gen(const T** arg, T** res, casadi_int* iw, T* w, int mem=0) const;

    /// Evaluate the function numerically
    int eval(const double** arg, double** res, casadi_int* iw, double* w, void* mem) const override;

    /// Type of parallellization
    virtual std::string parallelization() const { return "serial"; }

    /** \brief  evaluate symbolically while also propagating directional derivatives */
    int eval_sx(const SXElem** arg, SXElem** res,
                casadi_int* iw, SXElem* w, void* mem) const override;

    /** \brief  Propagate sparsity forward */
    int sp_forward(const bvec_t** arg, bvec_t** res,
                    casadi_int* iw, bvec_t* w, void* mem) const override;

    /** \brief  Propagate sparsity backwards */
    int sp_reverse(bvec_t** arg, bvec_t** res, casadi_int* iw, bvec_t* w, void* mem) const override;

    ///@{
    /// Is the class able to propagate seeds through the algorithm?
    bool has_spfwd() const override { return true;}
    bool has_sprev() const override { return true;}
    ///@}

    /** \brief Is codegen supported? */
    bool has_codegen() const override { return true;}

    /** \brief Generate code for the declarations of the C function */
    void codegen_declarations(CodeGenerator& g) const override;

    /** \brief Generate code for the body of the C function */
    void codegen_body(CodeGenerator& g) const override;

    /** \brief  Initialize */
    void init(const Dict& opts) override;

    ///@{
    /** \brief Generate a function that calculates \a nfwd forward derivatives */
    bool has_forward(casadi_int nfwd) const override { return true;}
    Function get_forward(casadi_int nfwd, const std::string& name,
                         const std::vector<std::string>& inames,
                         const std::vector<std::string>& onames,
                         const Dict& opts) const override;
    ///@}

    ///@{
    /** \brief Generate a function that calculates \a nadj adjoint derivatives */
    bool has_reverse(casadi_int nadj) const override { return true;}
    Function get_reverse(casadi_int nadj, const std::string& name,
                         const std::vector<std::string>& inames,
                         const std::vector<std::string>& onames,
                         const Dict& opts) const override;
    ///@}


    /** \brief Serialize an object without type information */
    void serialize_body(SerializingStream &s) const override;

    /** \brief Serialize type information */
    void serialize_type(SerializingStream &s) const override;

    /** \brief String used to identify the immediate FunctionInternal subclass */
    std::string serialize_base_function() const override { return "MapSum"; }

    /** \brief Deserialize with type disambiguation */
    static ProtoFunction* deserialize(DeserializingStream& s);

  protected:
    /** \brief Deserializing constructor */
    explicit MapSum(DeserializingStream& s);

    // Constructor (protected, use create function)
    MapSum(const std::string& name, const Function& f, casadi_int n,
           const std::vector<bool>& reduce_in,
           const std::vector<bool>& reduce_out);

    // The function which is to be evaluated in parallel
    Function f_;

    // Number of times to evaluate this function
    casadi_int n_;

    // Reduce an input?
    std::vector<bool> reduce_in_;

    // Reduce an output?
    std::vector<bool> reduce_out_;
  };


} // namespace casadi
/// \endcond

#endif // CASADI_MAPSUM_HPP
