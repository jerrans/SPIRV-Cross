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


#include "spirv_ispc.hpp"
#include "GLSL.std.450.h"
#include <assert.h>
#include <algorithm>
#include <iomanip> // std::put_time

#define PACKED ""
//#define PACKED "_packed"

using namespace spv;
using namespace spirv_cross;
using namespace std;

void CompilerISPC::emit_buffer_block(const SPIRVariable &var)
{
	add_resource_name(var.self);

	auto &type = get<SPIRType>(var.basetype);
	auto instance_name = to_name(var.self);

	uint32_t descriptor_set = meta[var.self].decoration.set;
	uint32_t binding = meta[var.self].decoration.binding;

	emit_block_struct(type);
	auto buffer_name = to_name(type.self);
    /*
	statement("internal::Resource<", buffer_name, type_to_array_glsl(type), "> ", instance_name, "__;");
	statement_no_indent("#define ", instance_name, " __res->", instance_name, "__.get()");
	resource_registrations.push_back(
	    join("s.register_resource(", instance_name, "__", ", ", descriptor_set, ", ", binding, ");"));
*/
    resource_registrations.push_back(join("uniform struct ", buffer_name, type_to_array_glsl(type), " ", instance_name, ";"));
    resource_entry_arguments.push_back(join("uniform struct ", buffer_name, type_to_array_glsl(type), "& ", instance_name));
    resource_entry_arguments_init.push_back(join(instance_name));

	statement("");
}

void CompilerISPC::emit_interface_block(const SPIRVariable &var)
{
	add_resource_name(var.self);

	auto &type = get<SPIRType>(var.basetype);

	const char *qual = var.storage == StorageClassInput ? "StageInput" : "StageOutput";
	const char *lowerqual = var.storage == StorageClassInput ? "stage_input" : "stage_output";
	auto instance_name = to_name(var.self);
	uint32_t location = meta[var.self].decoration.location;

	string buffer_name;
	auto flags = meta[type.self].decoration.decoration_flags;
	if (flags & (1ull << DecorationBlock))
	{
		emit_block_struct(type);
		buffer_name = to_name(type.self);
	}
	else
		buffer_name = type_to_glsl(type);

	statement("internal::", qual, "<", buffer_name, type_to_array_glsl(type), "> ", instance_name, "__;");
	statement_no_indent("#define ", instance_name, " __res->", instance_name, "__.get()");
	resource_registrations.push_back(join("s.register_", lowerqual, "(", instance_name, "__", ", ", location, ");"));
	statement("");
}

void CompilerISPC::emit_shared(const SPIRVariable &var)
{
	add_resource_name(var.self);

	auto instance_name = to_name(var.self);
	statement(CompilerGLSL::variable_decl(var), ";");
	statement_no_indent("#define ", instance_name, " __res->", instance_name);
}

void CompilerISPC::emit_uniform(const SPIRVariable &var)
{
	add_resource_name(var.self);

	auto &type = get<SPIRType>(var.basetype);
	auto instance_name = to_name(var.self);

	uint32_t descriptor_set = meta[var.self].decoration.set;
	uint32_t binding = meta[var.self].decoration.binding;
	uint32_t location = meta[var.self].decoration.location;

	string type_name = type_to_glsl(type);
	remap_variable_type_name(type, instance_name, type_name);

	if (type.basetype == SPIRType::AtomicCounter)
	{
		statement("internal::Resource<", type_name, type_to_array_glsl(type), "> ", instance_name, "__;");
//		statement_no_indent("#define ", instance_name, " __res->", instance_name, "__.get()");
//		resource_registrations.push_back(
//		    join("s.register_resource(", instance_name, "__", ", ", descriptor_set, ", ", binding, ");"));
	}
	else
	{
		statement("//unsupported internal::UniformConstant<", type_name, type_to_array_glsl(type), "> ", instance_name, "__;");
//		statement_no_indent("#define ", instance_name, " __res->", instance_name, "__.get()");
//		resource_registrations.push_back(
//		    join("s.register_uniform_constant(", instance_name, "__", ", ", location, ");"));
	}

	statement("");
}

void CompilerISPC::emit_push_constant_block(const SPIRVariable &var)
{
	add_resource_name(var.self);

	auto &type = get<SPIRType>(var.basetype);
	auto &flags = meta[var.self].decoration.decoration_flags;
	if ((flags & (1ull << DecorationBinding)) || (flags & (1ull << DecorationDescriptorSet)))
		SPIRV_CROSS_THROW("Push constant blocks cannot be compiled to GLSL with Binding or Set syntax. "
		                  "Remap to location with reflection API first or disable these decorations.");

	emit_block_struct(type);
	auto buffer_name = to_name(type.self);
	auto instance_name = to_name(var.self);

	statement("internal::PushConstant<", buffer_name, type_to_array_glsl(type), "> ", instance_name, ";");
	statement_no_indent("#define ", instance_name, " __res->", instance_name, ".get()");
	resource_registrations.push_back(join("s.register_push_constant(", instance_name, "__", ");"));
	statement("");
}

void CompilerISPC::emit_block_struct(SPIRType &type)
{
	// C++ can't do interface blocks, so we fake it by emitting a separate struct.
	// However, these structs are not allowed to alias anything, so remove it before
	// emitting the struct.
	//
	// The type we have here needs to be resolved to the non-pointer type so we can remove aliases.
	auto &self = get<SPIRType>(type.self);
	self.type_alias = 0;
	emit_struct(self);
}

void CompilerISPC::emit_resources()
{
    auto &execution = get_entry_point();

    vector<string> varyings = { "varying", "uniform" };

    if (requires_op_dot)
    {
        statement("");
        statement("//////////////////////////////");
        statement("// Dot Product");
        statement("//////////////////////////////");
        vector<vector<string>> varying_tuples = {
            { "varying", "varying", "varying" },
            { "varying", "varying", "uniform" },
            { "varying", "uniform", "varying" },
            { "uniform", "uniform", "uniform" },
        };
        for (auto &v : varying_tuples)
        {
            statement("inline ", v[0], " float dot(", v[1], " float4& lhs, ", v[2], " float4& rhs)");
            begin_scope();
            statement("return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;");
            end_scope();
            statement("");
            statement("inline ", v[0], " float dot(", v[1], " float3& lhs, ", v[2], " float3& rhs)");
            begin_scope();
            statement("return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;");
            end_scope();
            statement("");
            statement("inline ", v[0], " float dot(", v[1], " float2& lhs, ", v[2], " float2& rhs)");
            begin_scope();
            statement("return lhs.x * rhs.x + lhs.y * rhs.y;");
            end_scope();
            statement("");
        }
    }

    if (requires_op_len)
    {
        statement("");
        statement("//////////////////////////////");
        statement("// Length");
        statement("//////////////////////////////");
        for (auto &v : varyings)
        {
            statement("inline ", v, " float length(", v, " float3& rhs)");
            begin_scope();
            statement("return sqrt(dot(rhs, rhs));");
            end_scope();
            statement("");
        }
    }

    if (requires_op_reflect)
    {
        statement("");
        statement("//////////////////////////////");
        statement("// Reflect");
        statement("//////////////////////////////");
        vector<int> width = { 2, 3, 4 };
        vector<vector<string>> varying_tuples = {
            { "varying", "varying", "varying" },
            { "varying", "varying", "uniform" },
            { "varying", "uniform", "varying" },
            { "uniform", "uniform", "uniform" },
        };

        // v = i - 2 * n * dot(i•n)
        for (auto &v : varying_tuples)
        {
            for (auto &w : width)
            {
                statement("inline ", v[0], " float", w, " reflect(", v[1], " float", w, "& i, ", v[2], " float", w, "& n)");
                begin_scope();
                statement("return i - n * float", w, "_init(dot(i, n) * 2.0f);");
                end_scope();
                statement("");
            }
        }
    }

    if (requires_op_mix)
    {
        //x + s(y-x)
        statement("");
        statement("//////////////////////////////");
        statement("// Mix");
        statement("//////////////////////////////");
        vector<int> width = { 1, 2, 3, 4 };
        for (auto &v : varyings)
        {
            for (auto &w : width)
            {
                if (w == 1)
                    statement("inline ", v, " float mix(", v, " float& x, ", v, " float& y, ", v, " float& s)");
                else
                    statement("inline ", v, " float", w, " mix(", v, " float", w, "& x, ", v, " float", w, "& y, ", v, " float", w, "& s)");
                begin_scope();
                statement("return x + s * (y - x);");
                end_scope();
                statement("");
            }
        }
    }

    if (requires_op_atomics)
    {
        statement("");
        statement("//////////////////////////////");
        statement("// Atomics");
        statement("//////////////////////////////");
        vector<string> op = { "atomic_add", "atomic_subtract", "atomic_min", "atomic_max", "atomic_and", "atomic_or", "atomic_xor" };
        for (auto &o : op)
        {
            statement("inline varying int ", o, "(uniform int * uniform ptr, varying int value)");
            begin_scope();
            statement("uniform int ret[programCount];");
            statement("foreach_active(instance)");
            begin_scope();
            statement("uniform int val = extract(value, instance);");
            statement("ret[instance] = ", o, "_global(ptr, val);");
            end_scope();
            statement("varying int vRet = *((varying int * uniform) &ret);");
            statement("return vRet;");
            end_scope();
            statement("");

            statement("inline varying int ", o, "(uniform int * uniform ptr, uniform int value)");
            begin_scope();
            statement("uniform int ret[programCount];");
            statement("foreach_active(instance)");
            begin_scope();
            statement("ret[instance] = ", o, "_global(ptr, value);");
            end_scope();
            statement("varying int vRet = *((varying int * uniform) &ret);");
            statement("return vRet;");
            end_scope();
            statement("");
        }
    }

    // Output all basic struct types which are not Block or BufferBlock as these are declared inplace
	// when such variables are instantiated.
    statement("");
    statement("//////////////////////////////");
    statement("// Structs");
    statement("//////////////////////////////");
    for (auto &id : ids)
	{
		if (id.get_type() == TypeType)
		{
			auto &type = id.get<SPIRType>();
			if (type.basetype == SPIRType::Struct && type.array.empty() && !type.pointer &&
			    (meta[type.self].decoration.decoration_flags &
			     ((1ull << DecorationBlock) | (1ull << DecorationBufferBlock))) == 0)
			{
				emit_struct(type);
			}
		}
	}

//	statement("struct Resources : ", resource_type);
//	begin_scope();

	// Output UBOs and SSBOs
	for (auto &id : ids)
	{
		if (id.get_type() == TypeVariable)
		{
			auto &var = id.get<SPIRVariable>();
			auto &type = get<SPIRType>(var.basetype);

			if (var.storage != StorageClassFunction && type.pointer && type.storage == StorageClassUniform &&
			    !is_hidden_variable(var) && (meta[type.self].decoration.decoration_flags &
			                                 ((1ull << DecorationBlock) | (1ull << DecorationBufferBlock))))
			{
				emit_buffer_block(var);
			}
		}
	}

	// Output push constant blocks
	for (auto &id : ids)
	{
		if (id.get_type() == TypeVariable)
		{
			auto &var = id.get<SPIRVariable>();
			auto &type = get<SPIRType>(var.basetype);
			if (!is_hidden_variable(var) && var.storage != StorageClassFunction && type.pointer &&
			    type.storage == StorageClassPushConstant)
			{
				emit_push_constant_block(var);
			}
		}
	}

	// Output in/out interfaces.
	for (auto &id : ids)
	{
		if (id.get_type() == TypeVariable)
		{
			auto &var = id.get<SPIRVariable>();
			auto &type = get<SPIRType>(var.basetype);

			if (var.storage != StorageClassFunction && !is_hidden_variable(var) && type.pointer &&
			    (var.storage == StorageClassInput || var.storage == StorageClassOutput) &&
			    interface_variable_exists_in_entry_point(var.self))
			{
				emit_interface_block(var);
			}
		}
	}

	// Output Uniform Constants (values, samplers, images, etc).
	for (auto &id : ids)
	{
		if (id.get_type() == TypeVariable)
		{
			auto &var = id.get<SPIRVariable>();
			auto &type = get<SPIRType>(var.basetype);

			if (var.storage != StorageClassFunction && !is_hidden_variable(var) && type.pointer &&
			    (type.storage == StorageClassUniformConstant || type.storage == StorageClassAtomicCounter))
			{
				emit_uniform(var);
			}
		}
	}

	// Global variables.
	bool emitted = false;
	for (auto global : global_variables)
	{
		auto &var = get<SPIRVariable>(global);
		if (var.storage == StorageClassWorkgroup)
		{
			emit_shared(var);
			emitted = true;
		}
	}

	// Emit regular globals which are allocated per invocation.
	emitted = false;
    statement("");
    statement("//////////////////////////////");
    statement("// Globals");
    statement("//////////////////////////////");
    for (auto global : global_variables)
	{
		auto &var = get<SPIRVariable>(global);
		if (var.storage == StorageClassPrivate)
		{
			if (var.storage == StorageClassWorkgroup)
				emit_shared(var);
			else
				statement(CompilerGLSL::variable_decl(var), ";");
			emitted = true;
		}
	}

    statement("");

    statement("//////////////////////////////");
    statement("// Converted Code");
    statement("//////////////////////////////");
}

void CompilerISPC::find_vectorisation_variables()
{  
    VectorisationHandler handler(*this);
    traverse_all_reachable_opcodes(get<SPIRFunction>(entry_point), handler);
    
    handler.propogate_ispc_varyings_for_builtins();
}

string CompilerISPC::compile()
{
	// Force a classic "C" locale, reverts when function returns
	ClassicLocale classic_locale;

    // Convert the use of global variables to recursively-passed function parameters
    localize_global_variables();
    extract_global_variables_from_functions();

	// Do not deal with ES-isms like precision, older extensions and such.
	options.es = false;
	options.version = 450;
	backend.float_literal_suffix = true;
	backend.double_literal_suffix = false;
	backend.long_long_literal_suffix = true;
	backend.uint32_t_literal_suffix = true;
	backend.basic_int_type = "int";
    backend.basic_uint_type = "int"; // We should support this at some point...
    backend.swizzle_is_function = false;
	backend.shared_is_implied = true;
	backend.flexible_member_array_supported = false;
	backend.explicit_struct_type = true;
	backend.use_initializer_list = true;

	update_active_builtins();

    find_vectorisation_variables();

	uint32_t pass_count = 0;
	do
	{
        if (pass_count >= 3)
			SPIRV_CROSS_THROW("Over 3 compilation loops detected. Must be a bug!");

        resource_registrations.clear();
        resource_entry_arguments.clear();
        resource_entry_arguments_init.clear();
        reset();

		// Move constructor for this type is broken on GCC 4.9 ...
		buffer = unique_ptr<ostringstream>(new ostringstream());

		emit_header();
		emit_resources();

		emit_function(get<SPIRFunction>(entry_point), 0);

		pass_count++;
	} while (force_recompile);

	// Match opening scope of emit_header().
//	end_scope_decl();
	// namespace
//	end_scope();

	// Emit C entry points
	emit_c_linkage();

    emit_ispc_main();

	return buffer->str();
}

void CompilerISPC::emit_c_linkage()
{
    /*
	statement("");

	statement("spirv_cross_shader_t *spirv_cross_construct(void)");
	begin_scope();
	statement("return new ", impl_type, "();");
	end_scope();

	statement("");
	statement("void spirv_cross_destruct(spirv_cross_shader_t *shader)");
	begin_scope();
	statement("delete static_cast<", impl_type, "*>(shader);");
	end_scope();

	statement("");
	statement("void spirv_cross_invoke(spirv_cross_shader_t *shader)");
	begin_scope();
	statement("static_cast<", impl_type, "*>(shader)->invoke();");
	end_scope();

	statement("");
	statement("static const struct spirv_cross_interface vtable =");
	begin_scope();
	statement("spirv_cross_construct,");
	statement("spirv_cross_destruct,");
	statement("spirv_cross_invoke,");
	end_scope_decl();

	statement("");
	statement("const struct spirv_cross_interface *",
	          interface_name.empty() ? string("spirv_cross_get_interface") : interface_name, "(void)");
	begin_scope();
	statement("return &vtable;");
	end_scope();
    */
}

void CompilerISPC::emit_ispc_main()
{
    auto &execution = get_entry_point();

    string entry_point_name = interface_name.empty() ? string("ispc") : interface_name;


    statement("");
    statement("//////////////////////////////");
    statement("// ISPC Entry Points");
    statement("//////////////////////////////");

    // Dispatch all
    if (resource_entry_arguments.empty())
        statement("export void ", entry_point_name, "_dispatch_all(uniform int wg[3])");
    else
    {
        string decl = "export void " + entry_point_name + "_dispatch_all(uniform int wg[3], ";
        decl += entry_point_args(!get<SPIRFunction>(entry_point).arguments.empty(), false);
        decl += ")";

        statement(decl);
    }
    {
        begin_scope();

        statement("// Loop over the work group dimensions");
        statement("for(uniform int z = 0; z < wg[2]; z++)");
        {
            begin_scope();
            statement("for(uniform int y = 0; y < wg[1]; y++)");
            {
                begin_scope();
                statement("for(uniform int x = 0; x < wg[0]; x++)");
                {
                    begin_scope();
                    statement("uniform int3 gl_WorkGroupID = {x, y, z};"); //  
                    assert(execution.workgroup_size.y == 1);
                    assert(execution.workgroup_size.z == 1);
                    statement("");
                    statement("// Vectorise the workgroup");
                    statement("foreach(ii = 0 ... gl_WorkGroupSize.x )");
                    {
                        begin_scope();
                        statement("// Keep it 1-dimensional for now");
                        statement("varying int3 gl_GlobalInvocationID = {x * gl_WorkGroupSize.x + ii, y, z};");
                        statement("varying int3 gl_LocalInvocationID= {ii, y, z};"); // 
                        statement("varying int gl_LocalInvocationIndex = gl_LocalInvocationID.z * gl_WorkGroupSize.x * gl_WorkGroupSize.y + gl_LocalInvocationID.y * gl_WorkGroupSize.x + gl_LocalInvocationID.x;");
                        statement("");

                        if (resource_entry_arguments.empty())
                            statement("ispc_main();");
                        else
                        {
                            string decl = "ispc_main(";
                            decl += entry_point_args_init(!get<SPIRFunction>(entry_point).arguments.empty(), true);
                            decl += ");";
                            statement(decl);
/*
                            string args;
                            for (uint32_t ii = 0; ii < resource_entry_arguments_init.size(); ii++)
                            {
                                args += resource_entry_arguments_init[ii];
                                if ((ii + 1) < resource_entry_arguments_init.size())
                                    args += ", ";
                            }
                            statement("main(", args, ");");
*/
                        }


                        end_scope();
                    }
                    end_scope();
                }
                end_scope();
            }
            end_scope();
        }
        end_scope();
    }
    statement("");

    // Dispatch single
    if (resource_entry_arguments.empty())
        statement("export void ", "export void ", entry_point_name, "_dispatch_single(uniform int wg[3])");
    else
    {
        string decl = "export void " + entry_point_name + "_dispatch_single(uniform int wg[3], ";
        decl += entry_point_args(!get<SPIRFunction>(entry_point).arguments.empty(), false);
        decl += ")";

        statement(decl);
    }
    {
        begin_scope();
        statement("uniform int3 gl_WorkGroupID = {wg[0], wg[1], wg[2]};"); //  
        assert(execution.workgroup_size.y == 1);
        assert(execution.workgroup_size.z == 1);
        statement("");
        statement("// Vectorise the workgroup");
        statement("foreach(ii = 0 ... gl_WorkGroupSize.x )");
        {
            begin_scope();
            statement("// Keep it 1-dimensional for now");
            statement("varying int3 gl_GlobalInvocationID = {gl_WorkGroupID.x * gl_WorkGroupSize.x + ii, gl_WorkGroupID.y, gl_WorkGroupID.z};");
            statement("varying int3 gl_LocalInvocationID= {ii, gl_WorkGroupID.y, gl_WorkGroupID.z};"); // 
            statement("varying int gl_LocalInvocationIndex = gl_LocalInvocationID.z * gl_WorkGroupSize.x * gl_WorkGroupSize.y + gl_LocalInvocationID.y * gl_WorkGroupSize.x + gl_LocalInvocationID.x;");
            statement("");

            if (resource_entry_arguments.empty())
                statement("ispc_main();");
            else
            {
                string decl = "ispc_main(";
                decl += entry_point_args_init(!get<SPIRFunction>(entry_point).arguments.empty(), true);
                decl += ");";
                statement(decl);
            }
            end_scope();
        }
        end_scope();
    }

    statement("");
    statement("export void ", entry_point_name, "_get_workgroup_size(uniform int & wg_x, uniform int & wg_y, uniform int & wg_z)");
    begin_scope();
    statement("wg_x = gl_WorkGroupSize.x;");
    statement("wg_y = gl_WorkGroupSize.y;");
    statement("wg_z = gl_WorkGroupSize.z;");
    end_scope();
    statement("");

}

void CompilerISPC::emit_function_prototype(SPIRFunction &func, uint64_t)
{
	local_variable_names = resource_names;
	string decl;

	auto &type = get<SPIRType>(func.return_type);
	decl += "static inline ";
	decl += type_to_glsl(type);
	decl += " ";

	if (func.self == entry_point)
	{
		decl += "ispc_main";
		processing_entry_point = true;
	}
	else
		decl += to_name(func.self);

	decl += "(";

    if (processing_entry_point)
    {
        decl += entry_point_args(!func.arguments.empty(), true);
    }

	for (auto &arg : func.arguments)
	{
        //	add_local_variable_name(arg.id);

		decl += argument_decl(arg);
		if (&arg != &func.arguments.back())
			decl += ", ";

		// Hold a pointer to the parameter so we can invalidate the readonly field if needed.
		auto *var = maybe_get<SPIRVariable>(arg.id);
		if (var)
			var->parameter = &arg;
	}

	decl += ")";
	statement(decl);
}

string CompilerISPC::argument_decl(const SPIRFunction::Parameter &arg)
{
	auto &type = expression_type(arg.id);
	bool constref = !type.pointer || arg.write_count == 0;

    // remove const for now as it causes some ISPC issues
    constref = false;

	auto &var = get<SPIRVariable>(arg.id);

	string base = type_to_glsl(type, arg.id);
	string variable_name = to_name(var.self);
	remap_variable_type_name(type, variable_name, base);

	for (uint32_t i = 0; i < type.array.size(); i++)
		base = join("a std::array<", base, ", ", to_array_size(type, i), ">");

	return join(
        varyings[arg.id] ? "varying " : "uniform ", 
        constref ? "const " : "", 
        base, " &", variable_name);
}

string CompilerISPC::variable_decl(const SPIRType &type, const string &name, uint32_t id)
{
    string base;

    if (id > 0)
    {
        if (varyings[id])
            base = "varying ";
        else
            base = "uniform ";
    }
    else
    {
        // Should only get in here for struct members etc
    }

	base += type_to_glsl(type, id);
	remap_variable_type_name(type, name, base);
	bool runtime = false;
    string arrayString = "";

	for (uint32_t i = 0; i < type.array.size(); i++)
	{
		auto &array = type.array[i];
		if (!array && type.array_size_literal[i])
		{
			// Avoid using runtime arrays with std::array since this is undefined.
			// Runtime arrays cannot be passed around as values, so this is fine.
			runtime = true;
		}
        else
        {
            arrayString = join("[", to_array_size(type, i), "]");
        }
	}
	base += ' ';
	return base + name + (runtime ? "[]" : arrayString);
}

void CompilerISPC::create_default_constructor(std::string type, bool varying, uint32_t width, uint32_t arg_count, uint32_t arg_width[4])
{
    std::vector<string> arg_names = { "a", "b", "c", "d" };
    std::vector<string> arg_swizzles = { ".x", ".y", ".z", ".w" };

    std::vector<string> vector_names = { "varying ", "uniform " };
    std::string v = vector_names[varying ? 0 : 1];

    uint32_t function_count = 1;
    if (varying && arg_count > 1)
        function_count = 2;

    // For varying functions, create 2 variants
    // 1 - func (varying, varying)
    // 2 - func (varying, uniform)
    // Could do more, but that will suffice for now
    for (uint32_t ii = 0; ii < function_count; ii++)
    {
        // Create signature
        string args = "";
        for (uint32_t ac = 0; ac < arg_count; ac++)
        {
            if ((ac == arg_count - 1) && (function_count > 1))
                args += vector_names[ii];
            else
                args += v;

            args += join("const ", type);   // varying const float
            if (arg_width[ac] > 1)
                args += join(arg_width[ac]);        // varying const float2
            args += "&";                            // varying const float2&
            args += join(" ", arg_names[ac]);       // varying const float2& a
            if ((ac + 1) < arg_count)
                args += ", ";
        }

        statement("inline ", v, type, width, " ", type, width, "_init(", args, " )");
        begin_scope();

        // Create list initialiser
        string init;
        uint32_t required_initialisers = 0;
        while (required_initialisers < width)
        {
            for (uint32_t ac = 0; ac < arg_count; ac++)
            {
                for (uint32_t aw = 0; aw < arg_width[ac]; aw++)
                {
                    required_initialisers++;

                    init += arg_names[ac];
                    if (arg_width[ac] > 1)
                        init += arg_swizzles[aw];
                    if (required_initialisers < width)
                        init += ", ";
                }
            }
        }

        statement(v, type, width, " ret = { ", init, " }; ");
        statement("return ret;");
        end_scope();
        statement("");
    }
}

void CompilerISPC::create_default_binary_op(std::string type, uint32_t width, std::string op)
{
    vector<string> varying = { "uniform ", "varying " };
    vector<string> arg_swizzles = { ".x", ".y", ".z", ".w" };

    for (auto& arg1 : varying)
    {
        for (auto& arg2 : varying)
        {
            auto& ret_varying = (arg1 != arg2) ? varying[1] : arg1;
            // scalar
            statement("inline ", ret_varying, type, width, " operator", op, "(", arg1, type, width, " a, ", arg2, type, " b)");
            begin_scope();
            statement(ret_varying, type, width, " ret;");
            for (uint32_t ii = 0; ii < width; ii++)
            {
                statement("ret", arg_swizzles[ii], " = a", arg_swizzles[ii], " ", op, " b;");
            }
            statement("return ret;");
            end_scope();
            statement("");

            // vector
            statement("inline ", ret_varying, type, width, " operator", op, "(", arg1, type, width, " a, ", arg2, type, width, " b)");
            begin_scope();
            statement(ret_varying, type, width, " ret;");
            for (uint32_t ii = 0; ii < width; ii++)
            {
                statement("ret", arg_swizzles[ii], " = a", arg_swizzles[ii], " ", op, " b", arg_swizzles[ii], ";");
            }
            statement("return ret;");
            end_scope();
            statement("");
        }
    }
}

void CompilerISPC::create_default_load_op(std::string type, uint32_t width)
{
    vector<string> varying = { "uniform ", "varying " };
    vector<string> arg_swizzles = { ".x", ".y", ".z", ".w" };

    for (auto& v : varying)
    {
        statement("inline ", v , type, width, " load", width, "(", v, type, width, PACKED, " a)");
        begin_scope();
        string args;
        for (uint32_t ii = 0; ii < width; ii++)
        {
            args += join("a", arg_swizzles[ii]);
            if ((ii + 1) < width)
                args += ", ";
        }
        statement("return ", type, width, "_init(", args, ");");
        end_scope();
        statement("");
    }
}

void CompilerISPC::create_default_store_op(std::string type, uint32_t width)
{
    vector<string> varying = { "uniform ", "varying " };
    vector<string> arg_swizzles = { ".x", ".y", ".z", ".w" };
    for (auto& v : varying)
    {
        statement("inline ", v, type, width, PACKED, " store", width, "(", v, type, width, " a)");
        begin_scope();
        statement(v, type, width, PACKED, " ret;");

        string args;
        for (uint32_t ii = 0; ii < width; ii++)
        {
            statement("ret", arg_swizzles[ii], " = a", arg_swizzles[ii], ";");
        }
        statement("return ret;");
        end_scope();
        statement("");
    }
}

void CompilerISPC::create_default_structs(std::string type, uint32_t width)
{
    std::vector<string> arg_swizzles = { "x", "y", "z", "w" };
    statement("struct ", type, width, PACKED);
    begin_scope();
    string args;
    for (uint32_t ii = 0; ii < width; ii++)
    {
        statement(type, " ", arg_swizzles[ii], ";");
    }
    end_scope_decl();
    statement("");
}

void CompilerISPC::emit_header()
{
    auto &execution = get_entry_point();
    std::tm localTime;
    std::time_t result = std::time(nullptr);

    localtime_s(&localTime, &result);

    statement("//");
    statement("//////////////////////////////");
    statement("// This ISPC kernel is autogenerated by spirv-cross.");
    statement("// ", std::put_time(&localTime, "%c"));
    statement("//////////////////////////////");
    statement("//");
    statement("");

    statement("");
    statement("//////////////////////////////");
    statement("// Work Group");
    statement("//////////////////////////////");
    statement("static uniform int<3> gl_WorkGroupSize = {", execution.workgroup_size.x, ", ", execution.workgroup_size.y, ", ", execution.workgroup_size.z, "};");
    statement("");


    statement("");
    statement("//////////////////////////////");
    statement("// Default Types");
    statement("//////////////////////////////");
    statement("typedef float mat3[3][3];");
    statement("typedef float mat4[4][4];");

    std::vector<uint32_t> widths = { 2, 3, 4 };
    std::vector<string> types = { "float", "int" };
    for (const string& t : types)
    {
        create_default_structs(t, 1);
        for (const uint32_t& w : widths)
        {
            create_default_structs(t, w);
        }
    }

    std::vector<bool> varying = { true, false }; 
#if 1
    statement("");
    statement("//////////////////////////////");
    statement("// Default Constructors");
    statement("//////////////////////////////");
    for (const string& t : types)
    {
        for (const bool& v : varying)
        {
            for (const uint32_t& w : widths)
            {
                 uint32_t arg_widths[][4] = {
                    { 1, 1, 1, 1 }, // all
                    { 2, 1, 1, 1 }, // float 3 and larger
                    { 3, 1, 1, 1 }, // float 4 and larger

                };

                create_default_constructor(t, v, w, 1, arg_widths[0]);
                create_default_constructor(t, v, w, w, arg_widths[0]);

                if (w > 2)
                {
                    create_default_constructor(t, v, w, w - 1, arg_widths[1]);
                }
                if (w > 3)
                {
                    create_default_constructor(t, v, w, w - 2, arg_widths[2]);
                }
            }
        }
    }
#endif
    statement("");
    statement("//////////////////////////////");
    statement("// Default Operators");
    statement("//////////////////////////////");
    std::vector<uint32_t> load_widths = { 2, 3, 4 };
    std::vector<string> load_types = { "float", "int" };
//    std::vector<string> binary_ops = { "*", "/", "%", "+", "-" };
    std::vector<string> binary_ops = { "*", "/", "+", "-" };
    for (const string& t : load_types)
    {
        for (const uint32_t& w : load_widths)
        {
//            create_default_load_op(t, w);
//            create_default_store_op(t, w);
            for (const string& bop : binary_ops)
            {
                create_default_binary_op(t, w, bop);
            }
        }
    }


//	statement("#include \"spirv_cross/internal_interface.hpp\"");
//	statement("#include \"spirv_cross/external_interface.h\"");
	// Needed to properly implement GLSL-style arrays.
//	statement("#include <array>");
//	statement("#include <stdint.h>");
//	statement("");
//	statement("using namespace spirv_cross;");
//	statement("using namespace glm;");
//	statement("");

//	statement("namespace Impl");
//	begin_scope();

	switch (execution.model)
	{
	case ExecutionModelGeometry:
	case ExecutionModelTessellationControl:
	case ExecutionModelTessellationEvaluation:
	case ExecutionModelGLCompute:
	case ExecutionModelFragment:
	case ExecutionModelVertex:
//		statement("struct Shader");
//		begin_scope();
		break;

	default:
		SPIRV_CROSS_THROW("Unsupported execution model.");
	}

	switch (execution.model)
	{
	case ExecutionModelGeometry:
		impl_type = "GeometryShader<Impl::Shader, Impl::Shader::Resources>";
		resource_type = "GeometryResources";
		break;

	case ExecutionModelVertex:
		impl_type = "VertexShader<Impl::Shader, Impl::Shader::Resources>";
		resource_type = "VertexResources";
		break;

	case ExecutionModelFragment:
		impl_type = "FragmentShader<Impl::Shader, Impl::Shader::Resources>";
		resource_type = "FragmentResources";
		break;

	case ExecutionModelGLCompute:
		impl_type = join("ComputeShader<Impl::Shader, Impl::Shader::Resources, ", execution.workgroup_size.x, ", ",
		                 execution.workgroup_size.y, ", ", execution.workgroup_size.z, ">");
		resource_type = "ComputeResources";
		break;

	case ExecutionModelTessellationControl:
		impl_type = "TessControlShader<Impl::Shader, Impl::Shader::Resources>";
		resource_type = "TessControlResources";
		break;

	case ExecutionModelTessellationEvaluation:
		impl_type = "TessEvaluationShader<Impl::Shader, Impl::Shader::Resources>";
		resource_type = "TessEvaluationResources";
		break;

	default:
		SPIRV_CROSS_THROW("Unsupported execution model.");
	}
}

// The optional id parameter indicates the object whose type we are trying
// to find the description for. It is optional. Most type descriptions do not
// depend on a specific object's use of that type.
string CompilerISPC::type_to_glsl(const SPIRType &type, uint32_t id)
{
    // Ignore the pointer type since GLSL doesn't have pointers.

    switch (type.basetype)
    {
    case SPIRType::Struct:
        // Need OpName lookup here to get a "sensible" name for a struct.
        if (backend.explicit_struct_type)
            return join("struct ", to_name(type.self));
        else
            return to_name(type.self);

    case SPIRType::Image:
    case SPIRType::SampledImage:
        return image_type_glsl(type, id);

    case SPIRType::Sampler:
        // The depth field is set by calling code based on the variable ID of the sampler, effectively reintroducing
        // this distinction into the type system.
        return comparison_samplers.count(id) ? "samplerShadow" : "sampler";

    case SPIRType::Void:
        return "void";

    default:
        break;
    }

    if (type.vecsize == 1 && type.columns == 1) // Scalar builtin
    {
        switch (type.basetype)
        {
        case SPIRType::Boolean:
            return "bool";
        case SPIRType::Int:
            return backend.basic_int_type;
        case SPIRType::UInt:
            return backend.basic_uint_type;
        case SPIRType::AtomicCounter:
            return "atomic_uint";
        case SPIRType::Float:
            return "float";
        case SPIRType::Double:
            return "double";
        case SPIRType::Int64:
            return "int64";
        case SPIRType::UInt64:
            return "unsigned int64";
        default:
            return "???";
        }
    }
    else if (type.vecsize > 1 && type.columns == 1) // Vector builtin
    {
        // Its a struct, so can;t use short vectors if it is to be shared with the app
            if (id == 0)
            {
            switch (type.basetype)
            {
            case SPIRType::Boolean:
                return join("bool", type.vecsize, PACKED);
            case SPIRType::Int:
                return join("int", type.vecsize, PACKED);
            case SPIRType::UInt:
                return join("int", type.vecsize, PACKED);
            case SPIRType::Float:
                return join("float", type.vecsize, PACKED);
            case SPIRType::Double:
                return join("double", type.vecsize, PACKED);
            case SPIRType::Int64:
                return join("int64", type.vecsize, PACKED);
            case SPIRType::UInt64:
                return join("int64", type.vecsize, PACKED);
            default:
                return "???";
            }
        }
        else
        {
            switch (type.basetype)
            {
            case SPIRType::Boolean:
                return join("bool<", type.vecsize, ">");
            case SPIRType::Int:
                return join("int", type.vecsize);
            case SPIRType::UInt:
                return join("int", type.vecsize);
            case SPIRType::Float:
                return join("float", type.vecsize);
            case SPIRType::Double:
                return join("double<", type.vecsize, ">");
            case SPIRType::Int64:
                return join("int64<", type.vecsize, ">");
            case SPIRType::UInt64:
                return join("int64<", type.vecsize, ">");
            default:
                return "???";
            }
        }
    }
    else if (type.vecsize == type.columns) // Simple Matrix builtin
    {
        switch (type.basetype)
        {
        case SPIRType::Boolean:
            return join("bmat", type.vecsize);
        case SPIRType::Int:
            return join("imat", type.vecsize);
        case SPIRType::UInt:
            return join("umat", type.vecsize);
        case SPIRType::Float:
            return join("mat", type.vecsize);
        case SPIRType::Double:
            return join("dmat", type.vecsize);
            // Matrix types not supported for int64/uint64.
        default:
            return "???";
        }
    }
    else
    {
        switch (type.basetype)
        {
        case SPIRType::Boolean:
            return join("bmat", type.columns, "x", type.vecsize);
        case SPIRType::Int:
            return join("imat", type.columns, "x", type.vecsize);
        case SPIRType::UInt:
            return join("umat", type.columns, "x", type.vecsize);
        case SPIRType::Float:
            return join("mat", type.columns, "x", type.vecsize);
        case SPIRType::Double:
            return join("dmat", type.columns, "x", type.vecsize);
            // Matrix types not supported for int64/uint64.
        default:
            return "???";
        }
    }
}


void CompilerISPC::emit_instruction(const Instruction &instruction)
{
    auto ops = stream(instruction);
    auto opcode = static_cast<Op>(instruction.op);
    uint32_t length = instruction.length;

#define BOP(op) emit_binary_op(ops[0], ops[1], ops[2], ops[3], #op)
#define BOP_CAST(op, type) \
	emit_binary_op_cast(ops[0], ops[1], ops[2], ops[3], #op, type, opcode_is_sign_invariant(opcode))
#define UOP(op) emit_unary_op(ops[0], ops[1], ops[2], #op)
#define QFOP(op) emit_quaternary_func_op(ops[0], ops[1], ops[2], ops[3], ops[4], ops[5], #op)
#define TFOP(op) emit_trinary_func_op(ops[0], ops[1], ops[2], ops[3], ops[4], #op)
#define BFOP(op) emit_binary_func_op(ops[0], ops[1], ops[2], ops[3], #op)
#define BFOP_CAST(op, type) \
	emit_binary_func_op_cast(ops[0], ops[1], ops[2], ops[3], #op, type, opcode_is_sign_invariant(opcode))
#define BFOP(op) emit_binary_func_op(ops[0], ops[1], ops[2], ops[3], #op)
#define UFOP(op) emit_unary_func_op(ops[0], ops[1], ops[2], #op)

    switch (opcode)
    {
/*    case OpAccessChain:
    case OpInBoundsAccessChain:
    {
        emit_access_chain(instruction);
        break;
    }

    case OpStore:
    {
        emit_store(instruction);
        break;
    }

    case OpLoad:
    {
        emit_load(instruction);
        break;
    }
*/
    case OpDot:
    {
        if (!requires_op_dot)
        {
            requires_op_dot = true;
            force_recompile = true;
        }
        CompilerGLSL::emit_instruction(instruction);
        break;
    }

    // Conversion
    // ISPC doesn't like 
    //     float(val), 
    // it prefers 
    //    (float)val
    case OpConvertFToU:
    case OpConvertFToS:
    case OpConvertSToF:
    case OpConvertUToF:
    case OpUConvert:
    case OpSConvert:
    case OpFConvert:
    {
        uint32_t result_type = ops[0];
        uint32_t id = ops[1];

        auto func = type_to_glsl_constructor(get<SPIRType>(result_type));
        bool forward = should_forward(ops[2]);
        emit_op(result_type, id, join("(", func.c_str(), ")", to_expression(ops[2])), forward);
        inherit_expression_dependencies(id, ops[2]);

        break;
    }

    // ISPC currently has an issue with shuffles.
      // varying/uniform float<4> rhs;
      // varying float<3> lhs;
      // lhs = rhs.xyz  <-- This will currently cause ISPC to barf!
    //
    // We work around it by forcing all shuffles to go through the if(shuffle) path which uses constructor style code
      // lhs = float3_init(rhs); 
    case OpVectorShuffle:
    {
        uint32_t result_type = ops[0];
        uint32_t id = ops[1];
        uint32_t vec0 = ops[2];
        uint32_t vec1 = ops[3];
        const auto *elems = &ops[4];
        length -= 4;

        auto &type0 = expression_type(vec0);

        // fore shuffle to true to workaround ISPC bug
        bool shuffle = true;
        for (uint32_t i = 0; i < length; i++)
            if (elems[i] >= type0.vecsize)
                shuffle = true;

        string expr;
        bool trivial_forward;

        if (shuffle)
        {
            trivial_forward = !expression_is_forwarded(vec0) && !expression_is_forwarded(vec1);

            // Constructor style and shuffling from two different vectors.
            vector<string> args;
            for (uint32_t i = 0; i < length; i++)
            {
                if (elems[i] >= type0.vecsize)
                    args.push_back(join(to_enclosed_expression(vec1), ".", index_to_swizzle(elems[i] - type0.vecsize)));
                else
                    args.push_back(join(to_enclosed_expression(vec0), ".", index_to_swizzle(elems[i])));
            }
            expr += join(type_to_glsl_constructor(get<SPIRType>(result_type)), "(", merge(args), ")");
        }
        else
        {
            trivial_forward = !expression_is_forwarded(vec0);

            // We only source from first vector, so can use swizzle.
            expr += to_enclosed_expression(vec0);
            expr += ".";
            for (uint32_t i = 0; i < length; i++)
                expr += index_to_swizzle(elems[i]);
            if (backend.swizzle_is_function && length > 1)
                expr += "()";
        }

        // A shuffle is trivial in that it doesn't actually *do* anything.
        // We inherit the forwardedness from our arguments to avoid flushing out to temporaries when it's not really needed.
        emit_op(result_type, id, expr, should_forward(vec0) && should_forward(vec1), trivial_forward);
        break;
    }

    case OpAtomicIAdd:
    case OpAtomicISub:
    case OpAtomicSMin:
    case OpAtomicUMin:
    case OpAtomicSMax:
    case OpAtomicUMax:
    case OpAtomicAnd:
    case OpAtomicOr:
    case OpAtomicXor:
    {
        if (check_atomic_image(ops[2]))
        {
            SPIRV_CROSS_THROW("Atomic images not supported for ISPC.");
        }

        if (!requires_op_atomics)
        {
            requires_op_atomics = true;
            force_recompile = true;
        }

        string func;
        switch (opcode)
        {
        case OpAtomicIAdd:
            func = "atomic_add"; break;
        case OpAtomicISub:
            func = "atomic_subtract"; break;
        case OpAtomicSMin:
        case OpAtomicUMin:
            func = "atomic_min"; break;
        case OpAtomicSMax:
        case OpAtomicUMax:
            func = "atomic_max"; break;
        case OpAtomicAnd:
            func = "atomic_and"; break;
        case OpAtomicOr:
            func = "atomic_or"; break;
        case OpAtomicXor:
            func = "atomic_xor"; break;
        }

        forced_temporaries.insert(ops[1]);
        auto expr = join(func, "(&", to_expression(ops[2]), ", ", to_enclosed_expression(ops[5]), ")");
        emit_op(ops[0], ops[1], expr, should_forward(ops[2]) && should_forward(ops[5]));
        flush_all_atomic_capable_variables();
        register_read(ops[1], ops[2], should_forward(ops[2]));
        break;
    }

    default:
        CompilerGLSL::emit_instruction(instruction);
        break;
    }
}

string CompilerISPC::type_to_glsl_constructor(const SPIRType &type)
{
    if (type.array.size() > 1)
    {
        if (options.flatten_multidimensional_arrays)
            SPIRV_CROSS_THROW("Cannot flatten constructors of multidimensional array constructors, e.g. float[][]().");
        else if (!options.es && options.version < 430)
            require_extension("GL_ARB_arrays_of_arrays");
        else if (options.es && options.version < 310)
            SPIRV_CROSS_THROW("Arrays of arrays not supported before ESSL version 310.");
    }

    if (type.vecsize > 1 && type.columns == 1) // Scalar builtin
        if (type.basetype == SPIRType::Float)
            return join("float", type.vecsize, "_init");

    auto e = type_to_glsl(type);
    for (uint32_t i = 0; i < type.array.size(); i++)
        e += "[]";
    return e;
}

void CompilerISPC::emit_glsl_op(uint32_t result_type, uint32_t id, uint32_t eop, const uint32_t *args, uint32_t count)
{
    GLSLstd450 op = static_cast<GLSLstd450>(eop);

    switch (op)
    {
        // Vector math
    case GLSLstd450Length:
        emit_unary_func_op(result_type, id, args[0], "length");
        if (!requires_op_len)
        {
            requires_op_len = true;
            requires_op_dot = true;
            force_recompile = true;
        }
        break;

    case GLSLstd450Reflect:
        emit_binary_func_op(result_type, id, args[0], args[1], "reflect");
        if (!requires_op_reflect)
        {
            requires_op_reflect = true;
            requires_op_dot = true;
            force_recompile = true;
        }
        break;

    case GLSLstd450FMix:
        emit_trinary_func_op(result_type, id, args[0], args[1], args[2], "mix");
        if (!requires_op_mix)
        {
            requires_op_mix = true;
            force_recompile = true;
        }
        break;

    default:
        CompilerGLSL::emit_glsl_op(result_type, id, eop, args, count);
    }
}

bool CompilerISPC::VectorisationHandler::handle(spv::Op opcode, const uint32_t *args, uint32_t length)
{
    auto add_dependancies = [&](const uint32_t arg1, const uint32_t arg2)
    {
        dependee_hierarchy[arg2].insert(arg1);
    };

    switch (opcode)
    {
    case OpAccessChain:
    case OpInBoundsAccessChain:
    {
        if (length < 3)
            return false;

        // The access chain is name, type arg pairs. Need to add all dependancies
        for (uint32_t i = 2; i < length; i += 2)
        {
            add_dependancies(args[1], args[i]);

            // Access chains need to be 2-way as they are simply indirections.
            // But, if the src is a global passed in by the user, then we don't as they are always uniform
            // but perhaps with varying runtime arrays.
            auto * var = compiler.maybe_get<SPIRVariable>(args[i]);
            if (var && var->storage == StorageClassFunction)
            {
                add_dependancies(args[i], args[1]);
            }

        }

        break;
    }
    case OpCompositeExtract:
    case OpLoad:
    {
        if (length < 3)
            return false;

        add_dependancies(args[1], args[2]);
        break;
    }
    case OpStore:
    {
        if (length < 2)
            return false;

        add_dependancies(args[0], args[1]);
        break;
    }
    //    case OpFunctionParameter:
    case OpFunctionCall:
    {
        if (length < 3)
            return false;

        auto &func = compiler.get<SPIRFunction>(args[2]);
        const auto *arg = &args[3];
        length -= 3;

        // for functions, we need to add 2-way dependancies to ensure any
        // parameters promoted to varying within the function
        // are pushed back out of it.
        for (uint32_t i = 0; i < length; i++)
        {
            auto &argument = func.arguments[i];
            add_dependancies(argument.id, arg[i]);
            add_dependancies(arg[i], argument.id);
        }
        break;
    }

    case OpSNegate:
    case OpFNegate:
    case OpIAdd:
    case OpFAdd:
    case OpISub:
    case OpFSub:
    case OpIMul:
    case OpFMul:
    case OpUDiv:
    case OpSDiv:
    case OpFDiv:
    case OpVectorTimesScalar:
    case OpDot:
    case OpConvertSToF:
    case OpCompositeConstruct:
    {
        if (length < 3)
            return false;

        for (uint32_t i = 2; i < length; i++)
        {
            add_dependancies(args[1], args[i]);
        }
        break;
    }
    case OpVectorShuffle:
    {
        if (length < 4)
            return false;

        add_dependancies(args[1], args[2]);
        add_dependancies(args[1], args[3]);
        break;
    }
    case OpExtInst:
    {
        if (length < 5)
            return false;

        for (uint32_t i = 4; i < length; i++)
        {
            add_dependancies(args[1], args[i]);
        }
        break;
    }
    case OpAtomicISub:
    case OpAtomicIAdd:
    {
        // Atomics take uniform arguments (a buffer), but return varying results...
        compiler.varyings[args[1]] = true;
        break;
    }

    case OpUGreaterThan:
    case OpSGreaterThan:
    case OpUGreaterThanEqual:
    case OpSGreaterThanEqual:
    case OpULessThan:
    case OpSLessThan:
    case OpULessThanEqual:
    case OpSLessThanEqual:
    case OpFOrdEqual:
    case OpFUnordEqual:
    case OpFOrdNotEqual:
    case OpFUnordNotEqual:
    case OpFOrdLessThan:
    case OpFUnordLessThan:
    case OpFOrdGreaterThan:
    case OpFUnordGreaterThan:
    case OpFOrdLessThanEqual:
    case OpFUnordLessThanEqual:
    case OpFOrdGreaterThanEqual:
    case OpFUnordGreaterThanEqual:
    case OpLessOrGreater:
    case OpOrdered:
    case OpUnordered:
    case OpLogicalEqual:
    case OpLogicalNotEqual:
    case OpLogicalOr:
    case OpLogicalAnd:
    case OpLogicalNot:
        break;

    default:
    {
        // what is the default opcode
        printf("unknown opcode : %d\n", opcode);
    }
    }

    return true;
}

bool CompilerISPC::VectorisationHandler::propogate_ispc_varyings(const uint32_t var)
{
    bool bPropogate = false;
    auto &varIt = dependee_hierarchy.find(var);
    if (varIt != dependee_hierarchy.end())
    {
        if (compiler.varyings[var])
        {
            // walk all variables that depend on dependeeVar
            for (auto &dependantVarIt : dependee_hierarchy[var])
            {
                if (!compiler.varyings[dependantVarIt])
                {
                    compiler.varyings[dependantVarIt] = true;
                    bPropogate = true;

                    // Propogate to all variables dependant upon this variable
                    propogate_ispc_varyings(dependantVarIt);
//                    propogate_ispc_varyings_up(dependantVarIt);
                }
            }
        }
    }

    return bPropogate;
}

bool CompilerISPC::VectorisationHandler::propogate_ispc_varyings_for_builtins()
{
    bool bPropogate = false;
    for (auto &dependeeVarIt : dependee_hierarchy)
    {
        auto *dependeeVar = compiler.maybe_get<SPIRVariable>(dependeeVarIt.first);

        // Start with the builtins that we know need to be varyings.
        if (dependeeVar && compiler.is_builtin_variable(*dependeeVar))
        {
            switch (compiler.meta[dependeeVar->self].decoration.builtin_type)
            {
            case BuiltInLocalInvocationId:
            case BuiltInGlobalInvocationId:
            case BuiltInLocalInvocationIndex:
                compiler.varyings[dependeeVarIt.first] = true;
                bPropogate |= propogate_ispc_varyings(dependeeVarIt.first);
            }
        }
        // This will catch any pre-determined varying args.
        else if (compiler.varyings[dependeeVarIt.first])
        {
            bPropogate |= propogate_ispc_varyings(dependeeVarIt.first);
        }
    }

    return bPropogate;
}

// For any global variable accessed directly by a function,
// extract that variable and add it as an argument to that function.
void CompilerISPC::extract_global_variables_from_functions()
{

    // Uniforms
    unordered_set<uint32_t> global_var_ids;
    for (auto &id : ids)
    {
        if (id.get_type() == TypeVariable)
        {
            auto &var = id.get<SPIRVariable>();
            if (var.storage == StorageClassInput || var.storage == StorageClassUniform ||
                var.storage == StorageClassUniformConstant || var.storage == StorageClassPushConstant)
//            if (var.storage == StorageClassUniform)
            {
                global_var_ids.insert(var.self);
            }
        }
    }

    // Local vars that are declared in the main function and accessed directy by a function
    auto &entry_func = get<SPIRFunction>(entry_point);
    for (auto &var : entry_func.local_variables)
        global_var_ids.insert(var);

    std::set<uint32_t> added_arg_ids;
    unordered_set<uint32_t> processed_func_ids;
    extract_global_variables_from_function(entry_point, added_arg_ids, global_var_ids, processed_func_ids);
}

// MSL does not support the use of global variables for shader input content.
// For any global variable accessed directly by the specified function, extract that variable,
// add it as an argument to that function, and the arg to the added_arg_ids collection.
void CompilerISPC::extract_global_variables_from_function(uint32_t func_id, std::set<uint32_t> &added_arg_ids,
    unordered_set<uint32_t> &global_var_ids,
    unordered_set<uint32_t> &processed_func_ids)
{
    // Avoid processing a function more than once
    if (processed_func_ids.find(func_id) != processed_func_ids.end())
    {
        // Return function global variables
        added_arg_ids = function_global_vars[func_id];
        return;
    }

    processed_func_ids.insert(func_id);

    auto &func = get<SPIRFunction>(func_id);

    // Recursively establish global args added to functions on which we depend.
    for (auto block : func.blocks)
    {
        auto &b = get<SPIRBlock>(block);
        for (auto &i : b.ops)
        {
            auto ops = stream(i);
            auto op = static_cast<Op>(i.op);

            switch (op)
            {
            case OpLoad:
            case OpAccessChain:
            {
                uint32_t base_id = ops[2];
                if (global_var_ids.find(base_id) != global_var_ids.end())
                    added_arg_ids.insert(base_id);

                break;
            }
            case OpFunctionCall:
            {
                uint32_t inner_func_id = ops[2];
                std::set<uint32_t> inner_func_args;
                extract_global_variables_from_function(inner_func_id, inner_func_args, global_var_ids,
                    processed_func_ids);
                added_arg_ids.insert(inner_func_args.begin(), inner_func_args.end());
                break;
            }

            default:
                break;
            }
        }
    }

    function_global_vars[func_id] = added_arg_ids;

    // Add the global variables as arguments to the function
    if (func_id != entry_point)
    {
        uint32_t next_id = increase_bound_by(uint32_t(added_arg_ids.size()));
        for (uint32_t arg_id : added_arg_ids)
        {
            uint32_t type_id = get<SPIRVariable>(arg_id).basetype;
            func.add_parameter(type_id, next_id, true);
            set<SPIRVariable>(next_id, type_id, StorageClassFunction);

            // Ensure both the existing and new variables have the same name, and the name is valid
            string vld_name = ensure_valid_name(to_name(arg_id), "a");
            set_name(arg_id, vld_name);
            set_name(next_id, vld_name);

            meta[next_id].decoration.qualified_alias = meta[arg_id].decoration.qualified_alias;
            next_id++;
        }
    }
}

// Move the Private global variables to the entry function.
// Non-constant variables cannot have global scope in Metal.
void CompilerISPC::localize_global_variables()
{
    auto &entry_func = get<SPIRFunction>(entry_point);
    auto iter = global_variables.begin();
    while (iter != global_variables.end())
    {
        uint32_t gv_id = *iter;
        auto &gbl_var = get<SPIRVariable>(gv_id);
        if (gbl_var.storage == StorageClassPrivate)
        {
            entry_func.add_local_variable(gv_id);
            iter = global_variables.erase(iter);
        }
        else
            iter++;
    }
}

// Ensures that the specified name is permanently usable by prepending a prefix
// if the first chars are _ and a digit, which indicate a transient name.
string CompilerISPC::ensure_valid_name(string name, string pfx)
{
    if (name.size() >= 2 && name[0] == '_' && isdigit(name[1]))
    {
        return join(pfx, name);
    }
    else
    {
        return name;
    }
}

void CompilerISPC::find_entry_point_args()
{
    if (!entry_point_ids.empty())
        return;

    // Uniforms
    for (auto &id : ids)
    {
        if (id.get_type() == TypeVariable)
        {
            auto &var = id.get<SPIRVariable>();
            auto &type = get<SPIRType>(var.basetype);

            uint32_t var_id = var.self;

            if ((var.storage == StorageClassUniform || var.storage == StorageClassUniformConstant ||
                var.storage == StorageClassPushConstant) &&
                !is_hidden_variable(var))
            {
                switch (type.basetype)
                {
                case SPIRType::Struct:
                {
                    auto &m = meta.at(type.self);
                    if (m.members.size() == 0)
                        break;

                    entry_point_ids.push_back(&id);
                    break;
                }
                default:
                    break;
                }
            }
            if (var.storage == StorageClassInput && is_builtin_variable(var))
            {
                entry_point_ids.push_back(&id);
            }
        }
    }

    // Sort the args based on their names. This ensures that the ordering is consistant between
    // multiple runs of the compiler.
    if (entry_point_ids.size() > 1)
        sort(entry_point_ids.begin(), entry_point_ids.end(), 
            [=](const Variant* a, const Variant* b) -> bool
    {
        auto &varA = a->get<SPIRVariable>();
        auto &varB = b->get<SPIRVariable>();

        auto &nameA = to_name(varA.self);
        auto &nameB = to_name(varB.self);

        return nameA > nameB;
    }
    );
}


// Returns a string containing a comma-delimited list of args for the entry point function
string CompilerISPC::entry_point_args(bool append_comma, bool want_builtins)
{
    find_entry_point_args();

    string ep_args;

    // Uniforms
    for (auto &id : entry_point_ids)
    {
        auto &var = id->get<SPIRVariable>();
        auto &type = get<SPIRType>(var.basetype);

        uint32_t var_id = var.self;

        if ((var.storage == StorageClassUniform || var.storage == StorageClassUniformConstant ||
            var.storage == StorageClassPushConstant) &&
            !is_hidden_variable(var))
        {
            switch (type.basetype)
            {
            case SPIRType::Struct:
            {
                string varying = varyings[var_id] ? "varying " : "uniform ";
                auto &m = meta.at(type.self);
                if (m.members.size() == 0)
                    break;
                if (!ep_args.empty())
                    ep_args += ", ";
                ep_args += " " + varying + type_to_glsl(type, var_id) + "& " + to_name(var_id);
                break;
            }
            default:
                break;
            }
        }
        if (var.storage == StorageClassInput && is_builtin_variable(var) && want_builtins)
        {
            string varying = varyings[var_id] ? "varying " : "uniform ";
            if (!ep_args.empty())
                ep_args += ", ";
            BuiltIn bi_type = meta[var_id].decoration.builtin_type;
            ep_args += varying + type_to_glsl(type, var_id) + " " + to_expression(var_id);
        }
    }

    if (!ep_args.empty() && append_comma)
        ep_args += ", ";

    return ep_args;
}

string CompilerISPC::entry_point_args_init(bool append_comma, bool want_builtins)
{
    find_entry_point_args();

    string ep_args;
    // Uniforms
    for (auto &id : entry_point_ids)
    {
        auto &var = id->get<SPIRVariable>();
        auto &type = get<SPIRType>(var.basetype);

        uint32_t var_id = var.self;

        if ((var.storage == StorageClassUniform || var.storage == StorageClassUniformConstant ||
            var.storage == StorageClassPushConstant) &&
            !is_hidden_variable(var))
        {
            switch (type.basetype)
            {
            case SPIRType::Struct:
            {
                auto &m = meta.at(type.self);
                if (m.members.size() == 0)
                    break;
                if (!ep_args.empty())
                    ep_args += ", ";
                ep_args += " " + to_name(var_id);
                break;
            }
            default:
                break;
            }
        }
        if (var.storage == StorageClassInput && is_builtin_variable(var) && want_builtins)
        {
            if (!ep_args.empty())
                ep_args += ", ";
            BuiltIn bi_type = meta[var_id].decoration.builtin_type;
            ep_args += to_expression(var_id);
        }
    }

    if (!ep_args.empty() && append_comma)
        ep_args += ", ";

    return ep_args;
}

string CompilerISPC::read_access_chain(const SPIRAccessChain &chain)
{
    auto &type = get<SPIRType>(chain.basetype);

    SPIRType target_type;
    target_type.basetype = SPIRType::UInt;
    target_type.vecsize = type.vecsize;
    target_type.columns = type.columns;

    // FIXME: Transposition?
    if (type.columns != 1)
        SPIRV_CROSS_THROW("Reading matrices from ByteAddressBuffer not yet supported.");

    if (type.basetype == SPIRType::Struct)
        return "blurgh structs from ByteAddressBuffer";
//        SPIRV_CROSS_THROW("Reading structs from ByteAddressBuffer not yet supported.");

    if (type.width != 32)
        SPIRV_CROSS_THROW("Reading types other than 32-bit from ByteAddressBuffer not yet supported.");

    const char *load_op = nullptr;
    switch (type.vecsize)
    {
    case 1:
        load_op = "load1";
        break;
    case 2:
        load_op = "load2";
        break;
    case 3:
        load_op = "load3";
        break;
    case 4:
        load_op = "load4";
        break;
    default:
        SPIRV_CROSS_THROW("Unknown vector size.");
    }

    // currently have     varying float4 pos = uintBitsToFloat(oldPosVelo.Load4(DTid.x * 32 + 0));
    // want               varying float4 pos = float4_init(oldPosVelo._data[DTid.x].pos);

    //chain.base = oldPosVelo
    //chain.dynamic_index = DTid.x * 32;
    //chain.static_index = 0
    //bitcase = uintBitsToFloat

    auto load_expr = join(load_op, "(", chain.base, ")");

    return load_expr;
}

void CompilerISPC::emit_load(const Instruction &instruction)
{
    auto ops = stream(instruction);

    auto *chain = maybe_get<SPIRAccessChain>(ops[2]);
    if (chain)
    {
        uint32_t result_type = ops[0];
        uint32_t id = ops[1];
        uint32_t ptr = ops[2];

        auto load_expr = read_access_chain(*chain);

        bool forward = should_forward(ptr) && forced_temporaries.find(id) == end(forced_temporaries);
        auto &e = emit_op(result_type, id, load_expr, forward, true);
        e.need_transpose = false; // TODO: Forward this somehow.
        register_read(id, ptr, forward);
    }
    else
        CompilerGLSL::emit_instruction(instruction);
}

void CompilerISPC::emit_store(const Instruction &instruction)
{
    auto ops = stream(instruction);
    auto *chain = maybe_get<SPIRAccessChain>(ops[0]);
    auto &type = expression_type(ops[0]);
//    if (chain && type.vecsize > 1)
        if (0)
        {

        SPIRType target_type;
        target_type.basetype = SPIRType::UInt;
        target_type.vecsize = type.vecsize;
        target_type.columns = type.columns;

        const char *store_op = nullptr;
        switch (type.vecsize)
        {
        case 1:
            store_op = "store";
            break;
        case 2:
            store_op = "store2";
            break;
        case 3:
            store_op = "store3";
            break;
        case 4:
            store_op = "store4";
            break;
        default:
            SPIRV_CROSS_THROW("Unknown vector size.");
        }

        if (type.columns != 1)
            SPIRV_CROSS_THROW("Writing matrices to RWByteAddressBuffer not yet supported.");
        if (type.basetype == SPIRType::Struct)
            SPIRV_CROSS_THROW("Writing structs to RWByteAddressBuffer not yet supported.");
        if (type.width != 32)
            SPIRV_CROSS_THROW("Writing types other than 32-bit to RWByteAddressBuffer not yet supported.");

        // currently have         newPosVelo._data[DTid.x].pos.store4(DTid.x * 32 + 0, floatBitsToUint(pos));
        // want               newPosVelo._data[DTid.x].pos = store4(pos);

        //chain.base = newPosVelo

        auto store_expr = to_expression(ops[1]);
        statement(chain->base, " = ", store_op, "(", store_expr, ");");

        register_write(ops[0]);
    }
    else
        CompilerGLSL::emit_instruction(instruction);
}

void CompilerISPC::emit_access_chain(const Instruction &instruction)
{
    auto ops = stream(instruction);
    uint32_t length = instruction.length;

    bool need_byte_access_chain = false;
    auto &type = expression_type(ops[2]);
    const SPIRAccessChain *chain = nullptr;
    if (has_decoration(type.self, DecorationBufferBlock))
    {
        // If we are starting to poke into an SSBO, we are dealing with ByteAddressBuffers, and we need
        // to emit SPIRAccessChain rather than a plain SPIRExpression.
        uint32_t chain_arguments = length - 3;
        if (chain_arguments > type.array.size())
            need_byte_access_chain = true;
    }
    else
    {
        // Keep tacking on an existing access chain.
        chain = maybe_get<SPIRAccessChain>(ops[2]);
        if (chain)
            need_byte_access_chain = true;
    }

    if (need_byte_access_chain)
    {
        auto *var = maybe_get<SPIRVariable>(ops[2]);
        if (var)
            flush_variable_declaration(var->self);

        // If the base is immutable, the access chain pointer must also be.
        // If an expression is mutable and forwardable, we speculate that it is immutable.
        bool need_transpose;
        auto ac = access_chain(ops[2], &ops[3], length - 3, get<SPIRType>(ops[0]), &need_transpose);

        uint32_t to_plain_buffer_length = type.array.size();

        string base = ac;

        auto *basetype = &type;

        // Start traversing type hierarchy at the proper non-pointer types.
        while (basetype->pointer)
        {
            assert(basetype->parent_type);
            basetype = &get<SPIRType>(basetype->parent_type);
        }

        // Traverse the type hierarchy down to the actual buffer types.
        for (uint32_t i = 0; i < to_plain_buffer_length; i++)
        {
            assert(basetype->parent_type);
            basetype = &get<SPIRType>(basetype->parent_type);
        }
        uint32_t matrix_stride = 0;
        auto offsets =
            flattened_access_chain_offset(*basetype, &ops[3 + to_plain_buffer_length],
                length - 3 - to_plain_buffer_length, 0, 1, &need_transpose, &matrix_stride);

        auto &e = set<SPIRAccessChain>(ops[1], ops[0], type.storage, base, offsets.first, offsets.second);
        if (chain)
        {
            e.dynamic_index += chain->dynamic_index;
            e.static_index += chain->static_index;
        }

        e.immutable = should_forward(ops[2]);
    }
    else
    {
        CompilerGLSL::emit_instruction(instruction);
    }
}

string CompilerISPC::layout_for_member(const SPIRType &, uint32_t)
{
    return "";
}