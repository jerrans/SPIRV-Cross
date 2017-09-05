///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, Intel Corporation
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation 
// the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of 
// the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
// THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
// SOFTWARE.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
/*
 * Copyright 2015-2017 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SPIRV_CROSS_ISPC_HPP
#define SPIRV_CROSS_ISPC_HPP

#include "spirv_glsl.hpp"
#include <utility>
#include <vector>
#include <set>

namespace spirv_cross
{
class CompilerISPC : public CompilerGLSL
{
public:
	CompilerISPC(std::vector<uint32_t> spirv_)
	    : CompilerGLSL(move(spirv_))
	{
	}

	CompilerISPC(const uint32_t *ir, size_t word_count)
	    : CompilerGLSL(ir, word_count)
	{
	}

	std::string compile() override;

	// Sets a custom symbol name that can override
	// spirv_cross_get_interface.
	//
	// Useful when several shader interfaces are linked
	// statically into the same binary.
	void set_interface_name(std::string name)
	{
		interface_name = std::move(name);
	}

protected:
    // Specifically for the ISPC compiler
    // Look for potential targets to vectorise
    struct VectorisationHandler : OpcodeHandler
    {
        VectorisationHandler(CompilerISPC &compiler_)
            : compiler(compiler_)
        {
        }

        bool handle(spv::Op opcode, const uint32_t *args, uint32_t length) override;
        bool propogate_ispc_varyings_for_builtins();
        bool propogate_ispc_varyings(const uint32_t var);
        CompilerISPC &compiler;

        std::unordered_map<uint32_t, std::unordered_set<uint32_t>> dependee_hierarchy; // a = b; a is dependant upon b, so [b] = a

    };


private:
	void emit_header() override;
	void emit_c_linkage();
    void emit_ispc_main();
	void emit_function_prototype(SPIRFunction &func, uint64_t return_flags) override;

	void emit_resources();
	void emit_buffer_block(const SPIRVariable &type) override;
	void emit_push_constant_block(const SPIRVariable &var) override;
	void emit_interface_block(const SPIRVariable &type);
	void emit_block_chain(SPIRBlock &block);
	void emit_uniform(const SPIRVariable &var) override;
	void emit_shared(const SPIRVariable &var);
	void emit_block_struct(SPIRType &type);
    void emit_instruction(const Instruction &instruction) override;
    void emit_glsl_op(uint32_t result_type, uint32_t result_id, uint32_t op, const uint32_t *args,
        uint32_t count) override;
    std::string read_access_chain(const SPIRAccessChain &chain);
    void emit_load(const Instruction &instruction);
    void emit_store(const Instruction &instruction);
    void emit_access_chain(const Instruction &instruction);


    std::string variable_decl(const SPIRType &type, const std::string &name, uint32_t id) override;

	std::string argument_decl(const SPIRFunction::Parameter &arg);
    std::string type_to_glsl(const SPIRType &type, uint32_t id = 0) override;
    std::string type_to_glsl_constructor(const SPIRType &type) override;

    void find_vectorisation_variables();
    void create_default_constructor(std::string type, bool varying, uint32_t width, uint32_t arg_count, uint32_t arg_width[4]);
    void create_default_load_op(std::string type, uint32_t width);
    void create_default_store_op(std::string type, uint32_t width);
    void create_default_structs(std::string type, uint32_t width);
    void create_default_binary_op(std::string type, uint32_t width, std::string op);


    void extract_global_variables_from_functions();
    std::unordered_map<uint32_t, std::set<uint32_t>> function_global_vars;
    void extract_global_variables_from_function(uint32_t func_id, std::set<uint32_t> &added_arg_ids,
        std::unordered_set<uint32_t> &global_var_ids,
        std::unordered_set<uint32_t> &processed_func_ids);
    void localize_global_variables();
    std::string ensure_valid_name(std::string name, std::string pfx);
    std::string CompilerISPC::entry_point_args(bool append_comma, bool want_builtins);
    std::string CompilerISPC::entry_point_args_init(bool append_comma, bool want_builtins);
    void find_entry_point_args();
    std::string layout_for_member(const SPIRType &type, uint32_t index) override;
    bool optimize_read_modify_write(const std::string &lhs, const std::string &rhs) { return false; }

    std::vector<std::string> resource_registrations;
    std::vector<std::string> resource_entry_arguments;
    std::vector<std::string> resource_entry_arguments_init;

    std::vector<Variant*> entry_point_ids;
    
    std::string impl_type;
	std::string resource_type;
	uint32_t shared_counter = 0;

	std::string interface_name;

    bool requires_op_dot = false;
    bool requires_op_len = false;
    bool requires_op_reflect = false;
    bool requires_op_mix = false;
    bool requires_op_atomics = false;

    std::unordered_map<uint32_t, bool> varyings;

};
}

#endif
