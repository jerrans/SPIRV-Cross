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
#include <algorithm>
#include <assert.h>
#include <iomanip> // std::put_time

using namespace spv;
using namespace spirv_cross;
using namespace std;

#define DUMP_VARYING_DEPENDANCIES 1

void CompilerISPC::emit_buffer_block(const SPIRVariable &var)
{
	add_resource_name(var.self);

	auto &type = get<SPIRType>(var.basetype);
	auto instance_name = to_name(var.self);

	uint32_t descriptor_set = meta[var.self].decoration.set;
	uint32_t binding = meta[var.self].decoration.binding;

	emit_block_struct(type);
	auto buffer_name = to_name(type.self);
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
		//		    join("s.register_resource(", instance_name, "__", ", ", descriptor_set, ", ", binding, ");"));
	}
	else
	{
		//        statement("uniform ", type_name, " ", instance_name, type_to_array_glsl(type), ";");
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

void CompilerISPC::emit_struct(SPIRType &type)
{
	// Struct types can be stamped out multiple times
	// with just different offsets, matrix layouts, etc ...
	// Type-punning with these types is legal, which complicates things
	// when we are storing struct and array types in an SSBO for example.
	if (type.type_alias != 0)
		return;

	// Don't declare empty structs in GLSL, this is not allowed.
	// Empty structs is a corner case of HLSL output, and only sensible thing to do is avoiding to declare
	// these types.
	if (type_is_empty(type))
		return;

	add_resource_name(type.self);
	auto name = type_to_glsl(type);

	statement(!backend.explicit_struct_type ? "struct " : "", name);
	begin_scope();

	type.member_name_cache.clear();

	uint32_t i = 0;
	bool emitted = false;
	uint32_t max_size = 1;
	for (auto &member : type.member_types)
	{
		auto &membertype = get<SPIRType>(member);

		// if we are a runtime array, try and impose our size on the underlying type
		// We then need to pick that up and use it for padding
		if (!membertype.array.empty() && !membertype.array.back())
		{
			auto array_size = get_declared_struct_size(type);
			auto member_size = get_declared_struct_size(membertype);
			auto padding_size = array_size - member_size;
			if (padding_size && meta[membertype.self].decoration.runtime_array_padding != padding_size)
			{
				meta[membertype.self].decoration.runtime_array_padding = padding_size;
				force_recompile = true;
			}
		}

		add_member_name(type, i);
		emit_struct_member(type, member, i);
		i++;
		emitted = true;
	}

	if (emitted)
	{
		if (meta[type.self].decoration.runtime_array_padding)
		{
			statement("int8 spirvcross_std430_auto_padding[",
			          to_string(meta[type.self].decoration.runtime_array_padding), "];");
		}
	}
	end_scope_decl();

	if (emitted)
		statement("");
}

void CompilerISPC::emit_specialization_constants()
{
	bool emitted = false;
	for (auto &id : ids)
	{
		if (id.get_type() == TypeConstant)
		{
			auto &c = id.get<SPIRConstant>();
			if (!c.specialization)
				continue;

			auto &type = get<SPIRType>(c.constant_type);
			auto name = to_name(c.self);

			statement("#define ", name, " ", constant_expression(c));
			//            statement("const ", variable_decl(type, name, 0), " = ", constant_expression(c), ";");
			emitted = true;
		}
	}

	if (emitted)
		statement("");
}

string CompilerISPC::bitcast_glsl_op(const SPIRType &out_type, const SPIRType &in_type)
{
	// Ensure ISPC uses the (type) cast, rather than the type() cast.
	if (is_trivial_bitcast_glsl_op(out_type, in_type))
		return join("(" + type_to_glsl(out_type) + ")");

	return CompilerGLSL::bitcast_glsl_op(out_type, in_type);
}

void CompilerISPC::emit_resources()
{
	auto &execution = get_entry_point();

	vector<string> varyings = { "varying", "uniform" };

	// Output all basic struct types which are not Block or BufferBlock as these are declared inplace
	// when such variables are instantiated.
	statement("");
	statement("//////////////////////////////");
	statement("// Resources");
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

	// Specialisation constants
	emit_specialization_constants();

	// Output UBOs and SSBOs
	for (auto &id : ids)
	{
		if (id.get_type() == TypeVariable)
		{
			auto &var = id.get<SPIRVariable>();
			auto &type = get<SPIRType>(var.basetype);

			if (var.storage != StorageClassFunction && type.pointer && type.storage == StorageClassUniform &&
			    !is_hidden_variable(var) &&
			    (meta[type.self].decoration.decoration_flags &
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
			    (type.storage == StorageClassUniformConstant ||
			     type.storage == StorageClassAtomicCounter)) // || type.storage == StorageClassPrivate))
			{
				emit_uniform(var);
			}
		}
	}

	// Emit regular globals which are allocated per invocation.
	for (auto global : global_variables)
	{
		auto &var = get<SPIRVariable>(global);
		if (var.storage == StorageClassWorkgroup)
		{
			emit_shared(var);
		}
		else if (var.storage == StorageClassPrivate)
		{
			statement(CompilerGLSL::variable_decl(var), ";");
		}
	}

	statement("");

	statement("//////////////////////////////");
	statement("// Shader Code");
	statement("//////////////////////////////");
}

void CompilerISPC::find_vectorisation_variables()
{
	VectorisationHandler handler(*this);
	traverse_all_reachable_opcodes(get<SPIRFunction>(entry_point), handler);

	handler.propogate_ispc_varyings_for_builtins();
#if DUMP_VARYING_DEPENDANCIES
	handler.dump_varying_dependancies();
#endif
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
	backend.stdlib_filename = "spirvcross_stdlib.ispc";

	update_active_builtins();

	find_vectorisation_variables();

	uint32_t pass_count = 0;
	do
	{
		if (pass_count >= 3)
			SPIRV_CROSS_THROW("Over 3 compilation loops detected. Must be a bug!");

		reset();

		// Move constructor for this type is broken on GCC 4.9 ...
		// Write to the stdlib buffer, then swap the buffer pointers over.
		buffer = unique_ptr<ostringstream>(new ostringstream());
		emit_stdlib();
		stdlib_buffer = std::move(buffer);

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
	string decl = "export void " + entry_point_name + "_dispatch_all(uniform int work_groups[3]";
	string args = entry_point_args(!get<SPIRFunction>(entry_point).arguments.empty(), false);
	if (!args.empty())
		decl = join(decl, ", ", args);
	decl += ")";
	statement(decl);

	{
		begin_scope();

		statement("uniform int3 gl_NumWorkGroups = int3_init(work_groups[0], work_groups[1], work_groups[2]);");
		statement("");

		statement("// Loop over the work group dimensions");
		statement("for(uniform int z = 0; z < gl_NumWorkGroups.z; z++)");
		{
			begin_scope();
			statement("for(uniform int y = 0; y < gl_NumWorkGroups.y; y++)");
			{
				begin_scope();
				statement("for(uniform int x = 0; x < gl_NumWorkGroups.x; x++)");
				{
					begin_scope();
					statement("uniform int3 gl_WorkGroupID = int3_init(x, y, z);"); //
					statement("");
					statement("// Vectorise the workgroup");
					if (execution.workgroup_size.z > 1)
					{
						statement("foreach(lz = 0 ... gl_WorkGroupSize.z, ly = 0 ... gl_WorkGroupSize.y, "
						          "lx = 0 ... gl_WorkGroupSize.x)");
						begin_scope();
						statement("varying int3 gl_LocalInvocationID = int3_init(lx, ly, lz);");
					}
					else if (execution.workgroup_size.y > 1)
					{
						statement("foreach(ly = 0 ... gl_WorkGroupSize.y, lx = 0 ... gl_WorkGroupSize.x)");
						begin_scope();
						statement("varying int3 gl_LocalInvocationID = int3_init(lx, ly, 0);");
					}
					else
					{
						statement("foreach(lx = 0 ... gl_WorkGroupSize.x)");
						begin_scope();
						statement("varying int3 gl_LocalInvocationID = int3_init(lx, 0, 0);");
					}
					{
						statement("varying int3 gl_GlobalInvocationID = gl_WorkGroupID * gl_WorkGroupSize + "
						          "gl_LocalInvocationID;");
						statement("varying int gl_LocalInvocationIndex = gl_LocalInvocationID.z * gl_WorkGroupSize.x * "
						          "gl_WorkGroupSize.y + gl_LocalInvocationID.y * gl_WorkGroupSize.x + "
						          "gl_LocalInvocationID.x;");
						statement("");

						string decl = entry_point_name;
						decl += "_ispc_main(";
						decl += entry_point_args_init(!get<SPIRFunction>(entry_point).arguments.empty(), true);
						decl += ");";
						statement(decl);
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
	{
		string decl = "export void " + entry_point_name +
		              "_dispatch_single(uniform int work_group_ID[3], uniform int work_groups[3]";
		string args = entry_point_args(!get<SPIRFunction>(entry_point).arguments.empty(), false);
		if (!args.empty())
			decl = join(decl, ", ", args);
		decl += ")";
		statement(decl);
	}
	{
		begin_scope();
		statement("uniform int3 gl_NumWorkGroups = int3_init(work_groups[0], work_groups[1], work_groups[2]);");
		statement("uniform int3 gl_WorkGroupID = int3_init(work_group_ID[0], work_group_ID[1], work_group_ID[2]);");
		statement("");
		statement("// Vectorise the workgroup");
		if (execution.workgroup_size.z > 1)
		{
			statement("foreach(lz = 0 ... gl_WorkGroupSize.z, ly = 0 ... gl_WorkGroupSize.y, "
			          "lx = 0 ... gl_WorkGroupSize.x)");
			begin_scope();
			statement("varying int3 gl_LocalInvocationID = int3_init(lx, ly, lz);");
		}
		else if (execution.workgroup_size.y > 1)
		{
			statement("foreach(ly = 0 ... gl_WorkGroupSize.y, lx = 0 ... gl_WorkGroupSize.x)");
			begin_scope();
			statement("varying int3 gl_LocalInvocationID = int3_init(lx, ly, 0);");
		}
		else
		{
			statement("foreach(lx = 0 ... gl_WorkGroupSize.x)");
			begin_scope();
			statement("varying int3 gl_LocalInvocationID = int3_init(lx, 0, 0);");
		}
		{
			statement("varying int3 gl_GlobalInvocationID = gl_WorkGroupID * gl_WorkGroupSize + gl_LocalInvocationID;");
			statement("varying int gl_LocalInvocationIndex = gl_LocalInvocationID.z * gl_WorkGroupSize.x * "
			          "gl_WorkGroupSize.y + gl_LocalInvocationID.y * gl_WorkGroupSize.x + gl_LocalInvocationID.x;");
			statement("");

			string decl = entry_point_name;
			decl += "_ispc_main(";
			decl += entry_point_args_init(!get<SPIRFunction>(entry_point).arguments.empty(), true);
			decl += ");";
			statement(decl);

			end_scope();
		}
		end_scope();
	}

	statement("");
	statement("export void ", entry_point_name,
	          "_get_workgroup_size(uniform int & wg_x, uniform int & wg_y, uniform int & wg_z)");
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
	decl += "static SPIRV_INLINE ";
	decl += type_to_glsl(type);
	decl += " ";

	if (func.self == entry_point)
	{
		string entry_point_name = interface_name.empty() ? string("ispc") : interface_name;
		decl += entry_point_name;
		decl += "_ispc_main";
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
		variable_name = join(variable_name, "[", to_array_size(type, i), "]");

	// arrays get confused if passed by reference
	string passByRef = type.array.size() ? " " : "& ";

	return join(varyings[arg.id] ? "varying " : "uniform ", constref ? "const " : "", base, passByRef, variable_name);
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
	statement("#include \"", backend.stdlib_filename, "\"");

	statement("");
	statement("//////////////////////////////");
	statement("// Work Group");
	statement("//////////////////////////////");
	statement("static uniform int3 gl_WorkGroupSize = {", execution.workgroup_size.x, ", ", execution.workgroup_size.y,
	          ", ", execution.workgroup_size.z, "};");
	statement("");

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
				return join("bool", type.vecsize);
			case SPIRType::Int:
				return join("int", type.vecsize);
			case SPIRType::UInt:
				return join("int", type.vecsize);
			case SPIRType::Float:
				return join("float", type.vecsize);
			case SPIRType::Double:
				return join("double", type.vecsize);
			case SPIRType::Int64:
				return join("int64", type.vecsize);
			case SPIRType::UInt64:
				return join("int64", type.vecsize);
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
	case OpInBoundsAccessChain:
	case OpAccessChain:
	{
		auto *var = maybe_get<SPIRVariable>(ops[2]);
		if (var)
			flush_variable_declaration(var->self);

		// If the base is immutable, the access chain pointer must also be.
		// If an expression is mutable and forwardable, we speculate that it is immutable.
		bool need_transpose;
		auto e = access_chain(ops[2], &ops[3], length - 3, get<SPIRType>(ops[0]), &need_transpose);
		auto &expr = set<SPIRExpression>(ops[1], move(e), ops[0], should_forward(ops[2]));
		expr.loaded_from = ops[2];
		expr.need_transpose = need_transpose;

		// ISPC needs access chains to be flagged as mutable.
		// This will force SPIRV-Cross to assign access chains to temporary variables and not forward them.
		// This is required for the case of :
		//   varying float4 = float4(ubo.array[idx].pos, 1.0)
		// where idx is a varying and ubo.array[] is a user provided uniform pointer to a runtime array,
		// and float4(varying float3, uniform float) requires a varying float3 as input.
		// As ubo.array[idx].pos is not in contiguous memory, it is not a varying and can not be used directly.
		// ISPC will throw an error about lvalues.
		// TODO : only use if the expression involves a runtime array
		expr.immutable = false;
		break;
	}

	case OpDot:
	{
		CompilerGLSL::emit_instruction(instruction);
		break;
	}

	// ISPC doesn't like
	//     float(val),
	// it prefers
	//    (float)val
	case OpBitcast:
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
		uint32_t arg = ops[2];

		// Bitcasts can be used for casting between an int and a uint.
		// bitcast_glsl_op() treats this as a simple cast of the form int(val),
		// but ISPC needs (int)val, so we go down the normal cast path for that.
		if ((opcode == OpBitcast) && !is_trivial_bitcast_glsl_op(get<SPIRType>(result_type), expression_type(arg)))
		{
			// Normal bit cast
			auto op = bitcast_glsl_op(get<SPIRType>(result_type), expression_type(arg));
			emit_unary_func_op(result_type, id, arg, op.c_str());
			break;
		}
		auto type = get<SPIRType>(result_type);
		auto func = type_to_glsl_constructor(type);
		bool forward = should_forward(ops[2]);
		if (type.vecsize > 1 && type.columns == 1) // Scalar builtin, so no cast required
			emit_op(result_type, id, join(func.c_str(), "(", to_expression(ops[2]), ")"), forward);
		else
			emit_op(result_type, id, join("(", func.c_str(), ")(", to_expression(ops[2]), ")"), forward);
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
	case OpAtomicExchange:
	{
		if (check_atomic_image(ops[2]))
		{
			SPIRV_CROSS_THROW("Atomic images not supported for ISPC.");
		}

		string func;
		switch (opcode)
		{
		case OpAtomicIAdd:
			func = "atomic_add";
			break;
		case OpAtomicISub:
			func = "atomic_subtract";
			break;
		case OpAtomicSMin:
		case OpAtomicUMin:
			func = "atomic_min";
			break;
		case OpAtomicSMax:
		case OpAtomicUMax:
			func = "atomic_max";
			break;
		case OpAtomicAnd:
			func = "atomic_and";
			break;
		case OpAtomicOr:
			func = "atomic_or";
			break;
		case OpAtomicXor:
			func = "atomic_xor";
			break;
		case OpAtomicExchange:
			func = "atomic_swap";
			break;
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
	{
		if (type.basetype == SPIRType::Float)
			return join("float", type.vecsize, "_init");
		if (type.basetype == SPIRType::Int)
			return join("int", type.vecsize, "_init");
		if (type.basetype == SPIRType::UInt)
			return join("int", type.vecsize, "_init");
	}

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
		break;

	case GLSLstd450Reflect:
		emit_binary_func_op(result_type, id, args[0], args[1], "reflect");
		break;

	case GLSLstd450FMix:
		emit_trinary_func_op(result_type, id, args[0], args[1], args[2], "mix");
		break;

	case GLSLstd450RoundEven:
		emit_unary_func_op(result_type, id, args[0], "round");
		break;

	case GLSLstd450InverseSqrt:
		emit_unary_func_op(result_type, id, args[0], "rsqrt");
		break;

	default:
		CompilerGLSL::emit_glsl_op(result_type, id, eop, args, count);
	}
}

bool CompilerISPC::VectorisationHandler::handle(spv::Op opcode, const uint32_t *args, uint32_t length)
{
	// dst = src; dst is dependant upon src, so [src] = dst
	auto add_dependancies = [&](const uint32_t dst, const uint32_t src) { dependee_hierarchy[src].insert(dst); };

	switch (opcode)
	{
	case OpAccessChain:
	case OpInBoundsAccessChain:
	{
		if (length < 3)
			return false;

		// The access chain is name, type, arg indices. Need to add all dependancies
		for (uint32_t i = 2; i < length; i++)
		{
			add_dependancies(args[1], args[i]);

			// Access chains need to be 2-way as they are simply indirections.
			// But, if the src is a global passed in by the user, then we don't as they are always uniform
			// but perhaps with varying runtime arrays.
			auto *var = compiler.maybe_get<SPIRVariable>(args[i]);
			if (var && var->storage == StorageClassFunction)
			{
				add_dependancies(args[i], args[1]);
			}
		}

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

		// Need to add a dependancy on the function return variable and the return of the last block in the function
		// This ensures that any variables returned and stored have their varying status correctly propogated
		// args[0] stores the return type
		// args[1] stores the return id

		// walk all blocks looking for return types and propogate to the function
		for (auto &b : func.blocks)
		{
			auto block = compiler.get<SPIRBlock>(b);

			// OpReturnValue can return Undef, so don't add a dependancy for this
			if (compiler.ids.at(block.return_value).get_type() != TypeUndef)
			{
				add_dependancies(args[1], block.return_value);
			}
		}
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
	case OpAtomicExchange:
	{
		// Atomics take uniform arguments (a buffer), but return varying results...
		compiler.varyings[args[1]] = true;
		break;
	}

	default:
	{
		for (uint32_t i = 2; i < length; i++)
		{
			add_dependancies(args[1], args[i]);
		}
		break;
	}
	}
	return true;
}

void CompilerISPC::VectorisationHandler::set_current_block(const SPIRBlock &block)
{
	// We track the Phi variables here and add the appropriate dependancies
	for (auto &phi : block.phi_variables)
	{
		dependee_hierarchy[phi.local_variable].insert(phi.function_variable);
	}
}

bool CompilerISPC::VectorisationHandler::propogate_ispc_varyings(const uint32_t var)
{
	bool bPropogate = false;
	auto &varIt = dependee_hierarchy.find(var);
	if (varIt != dependee_hierarchy.end())
	{
		if (compiler.varyings[var])
		{
			// walk all variables that depend on var
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

void CompilerISPC::VectorisationHandler::dump_varying_dependancies()
{
	fprintf(stderr, "============\n\n");
	fprintf(stderr, "Varying Dependancees\n\n");
	for (auto &dependeeVarIt : dependee_hierarchy)
	{
		fprintf(stderr, "%s [%d] = %d\n", compiler.to_name(dependeeVarIt.first).c_str(), dependeeVarIt.first,
		        compiler.varyings[dependeeVarIt.first]);
		if (compiler.varyings[dependeeVarIt.first])
		{
			fprintf(stderr, "\t: ");
			// walk all variables that depend on dependeeVar
			for (auto &dependantVarIt2 : dependee_hierarchy[dependeeVarIt.first])
			{
				fprintf(stderr, "%d@%d, ", dependantVarIt2, compiler.varyings[dependantVarIt2]);
			}
			fprintf(stderr, "\n");
		}
	}
	fprintf(stderr, "============\n\n");
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
				case SPIRType::Image:
				{
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
		sort(entry_point_ids.begin(), entry_point_ids.end(), [=](const Variant *a, const Variant *b) -> bool {
			auto &varA = a->get<SPIRVariable>();
			auto &varB = b->get<SPIRVariable>();

			auto &nameA = to_name(varA.self);
			auto &nameB = to_name(varB.self);

			return nameA > nameB;
		});
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
			case SPIRType::Image:
				if (!ep_args.empty())
					ep_args += ", ";
				ep_args += "uniform " + type_to_glsl(type, var_id) + " &" + to_name(var_id);
				//                ep_args += " [[texture(" + convert_to_string(get_metal_resource_index(var, type.basetype)) + ")]]";
				break;
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
			case SPIRType::Image:
			{
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

void CompilerISPC::codegen_default_binary_op(std::string type, uint32_t width, std::string op)
{
	vector<string> varying = { "uniform ", "varying " };
	vector<string> arg_swizzles = { ".x", ".y", ".z", ".w" };

	for (auto &arg1 : varying)
	{
		for (auto &arg2 : varying)
		{
			auto &ret_varying = (arg1 != arg2) ? varying[1] : arg1;

			// scalar
			std::string s = join("SPIRV_INLINE ", ret_varying, type, width, " operator", op, "(", arg1, type, width,
			                     " a, ", arg2, type, " b) { ");
			s += join(ret_varying, type, width, " ret = { ");
			for (uint32_t ii = 0; ii < width; ii++)
			{
				s += join("a", arg_swizzles[ii], " ", op, " b ");
				if ((ii + 1) < width)
					s += ", ";
			}
			s += join("}; return ret; }");
			statement(s);

			// vector
			s = join("SPIRV_INLINE ", ret_varying, type, width, " operator", op, "(", arg1, type, width, " a, ", arg2,
			         type, width, " b) { ");
			s += join(ret_varying, type, width, " ret = { ");
			for (uint32_t ii = 0; ii < width; ii++)
			{
				s += join("a", arg_swizzles[ii], " ", op, " b", arg_swizzles[ii], " ");
				if ((ii + 1) < width)
					s += ", ";
			}
			s += join("}; return ret; }");
			statement(s);
		}
	}
	statement("");
}

void CompilerISPC::codegen_load_op(std::string type, uint32_t width)
{
	vector<string> varying = { "uniform ", "varying " };
	vector<string> arg_swizzles = { ".x", ".y", ".z", ".w" };

	for (auto &v : varying)
	{
		statement("SPIRV_INLINE ", v, type, width, " load", width, "(", v, type, width, " a)");
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

void CompilerISPC::codegen_store_op(std::string type, uint32_t width)
{
	vector<string> varying = { "uniform ", "varying " };
	vector<string> arg_swizzles = { ".x", ".y", ".z", ".w" };
	for (auto &v : varying)
	{
		statement("SPIRV_INLINE ", v, type, width, " store", width, "(", v, type, width, " a)");
		begin_scope();
		statement(v, type, width, " ret;");

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

void CompilerISPC::codegen_default_structs(std::string type, uint32_t width)
{
	std::vector<string> arg_swizzles = { "x", "y", "z", "w" };
	statement("struct ", type, width);
	begin_scope();
	string args;
	for (uint32_t ii = 0; ii < width; ii++)
	{
		statement(type, " ", arg_swizzles[ii], ";");
	}
	end_scope_decl();
	statement("");
}

void CompilerISPC::codegen_default_pixel_structs(uint32_t width)
{
	std::vector<string> arg_swizzles = { "r", "g", "b", "a" };
	statement("struct pixel", width, "D");
	begin_scope();
	string args;
	for (uint32_t ii = 0; ii < width; ii++)
	{
		statement("unsigned int8 ", arg_swizzles[ii], ";");
	}
	end_scope_decl();
	statement("");
}

void CompilerISPC::codegen_default_image_structs(uint32_t width)
{
	statement("struct image", width, "D");
	begin_scope();
	statement("unsigned int width;");
	statement("unsigned int height;");
	statement("unsigned int stride;");
	statement("struct pixel4D* data;");
	end_scope_decl();
	statement("");
}

void CompilerISPC::codegen_constructor(std::string type, bool varying, uint32_t width, uint32_t arg_count,
                                       uint32_t arg_width[4])
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

			args += join("const ", type); // varying const float
			if (arg_width[ac] > 1)
			{
				args += join(arg_width[ac]); // varying const float2
				args += "&"; // varying const float2&
			}
			args += join(" ", arg_names[ac]); // varying const float2& a
			if ((ac + 1) < arg_count)
				args += ", ";
		}

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
		statement("SPIRV_INLINE ", v, type, width, " ", type, width, "_init(", args, " ) { ", v, type, width,
		          " ret = { ", init, " }; return ret; }");
	}
}

// varyings/vector widths are : return, arg1, arg2, arg3
void CompilerISPC::codegen_ternary_float_op_multiple_widths(
    std::string func_name, std::vector<std::vector<std::string>> &varyings,
    std::vector<std::vector<uint32_t>> &vector_widths,
    const std::function<void(std::vector<std::string> varyings, std::vector<uint32_t> vector_widths)> &func)
{
	statement("//////////////////////////////");
	statement("// ", func_name);
	statement("//////////////////////////////");

	for (auto &w : vector_widths)
	{
		for (auto &v : varyings)
		{
			// pass by ref if non POD
			std::string op = join("SPIRV_INLINE ", v[0], " float");
			if (w[0] > 1)
				op += join(w[0]);
			op += join(" ", func_name, "(", v[1], " float");
			if (w[1] > 1)
				op += join(w[1], "&");
			op += join(" a, ", v[2], " float");
			if (w[2] > 1)
				op += join(w[2], "&");
			op += join(" b, ", v[3], " float");
			if (w[3] > 1)
				op += join(w[3], "&");
			op += join(" c)");

			statement(op);
			begin_scope();
			func(v, w);
			end_scope();
			statement("");
		}
	}
};

// varyings/vector widths are : return, arg1, arg2, arg3
void CompilerISPC::codegen_ternary_float_op(
    std::string func_name, std::vector<std::vector<std::string>> &varyings, std::vector<uint32_t> &vector_width,
    const std::function<void(std::vector<std::string> varyings, uint32_t vector_width)> &func)
{
	statement("//////////////////////////////");
	statement("// ", func_name);
	statement("//////////////////////////////");

	for (auto &w : vector_width)
	{
		for (auto &v : varyings)
		{
			if (w == 1)
				statement("SPIRV_INLINE ", v[0], " float ", func_name, "(", v[1], " float a, ", v[2], " float b, ",
				          v[3], " float c)");
			else
				statement("SPIRV_INLINE ", v[0], " float", w, " ", func_name, "(", v[1], " float", w, "& a, ", v[2],
				          " float", w, "& b, ", v[3], " float", w, "& c)");

			begin_scope();
			func(v, w);
			end_scope();
			statement("");
		}
	}
};

// varyings/vector widths are : return, arg1, arg2
void CompilerISPC::codegen_ternary_float_op_simple(std::string func_name,
                                                   std::vector<std::vector<std::string>> &varyings,
                                                   std::vector<uint32_t> &vector_width)
{
	codegen_ternary_float_op(
	    func_name, varyings, vector_width, [&](std::vector<std::string> varyings, int vector_width) {
		    std::string s;
		    if (vector_width == 1)
			    s = join(varyings[0], " float ret = { ", func_name, "(a.x, b.x, c.x)");
		    else
			    s = join(varyings[0], " float", vector_width, " ret = { ", func_name, "(a.x, b.x, c.x)");
		    if (vector_width > 1)
			    s += join(", ", func_name, "(a.y, b.y, c.y)");
		    if (vector_width > 2)
			    s += join(", ", func_name, "(a.z, b.z, c.z)");
		    if (vector_width > 3)
			    s += join(", ", func_name, "(a.w, b.w, c.w)");
		    s += " }; return ret;";
		    statement(s);
	    });
};

// varyings/vector widths are : return, arg1, arg2, arg3
void CompilerISPC::codegen_ternary_float_op_scalar_return(
    std::string func_name, std::vector<std::vector<std::string>> &varyings, std::vector<uint32_t> &vector_width,
    const std::function<void(std::vector<std::string> varyings, uint32_t vector_width)> &func)
{
	statement("//////////////////////////////");
	statement("// ", func_name);
	statement("//////////////////////////////");

	for (auto &w : vector_width)
	{
		for (auto &v : varyings)
		{
			if (w == 1)
				statement("SPIRV_INLINE ", v[0], " float ", func_name, "(", v[1], " float a, ", v[2], " float b, ",
				          v[3], " float c)");
			else
				statement("SPIRV_INLINE ", v[0], " float ", func_name, "(", v[1], " float", w, "& a, ", v[2], " float",
				          w, "& b, ", v[3], " float", w, "& c)");

			begin_scope();
			func(v, w);
			end_scope();
			statement("");
		}
	}
};

// varyings/vector widths are : return, arg1, arg2
void CompilerISPC::codegen_binary_float_op(
    std::string func_name, std::vector<std::vector<std::string>> &varyings, std::vector<uint32_t> &vector_width,
    const std::function<void(std::vector<std::string> varyings, uint32_t vector_width)> &func)
{
	statement("//////////////////////////////");
	statement("// ", func_name);
	statement("//////////////////////////////");

	for (auto &w : vector_width)
	{
		for (auto &v : varyings)
		{
			if (w == 1)
				statement("SPIRV_INLINE ", v[0], " float ", func_name, "(", v[1], " float a, ", v[2], " float b)");
			else
				statement("SPIRV_INLINE ", v[0], " float", w, " ", func_name, "(", v[1], " float", w, "& a, ", v[2],
				          " float", w, "& b)");

			begin_scope();
			func(v, w);
			end_scope();
			statement("");
		}
	}
};

// varyings/vector widths are : return, arg1, arg2
void CompilerISPC::codegen_binary_float_op_simple(std::string func_name,
                                                  std::vector<std::vector<std::string>> &varyings,
                                                  std::vector<uint32_t> &vector_width)
{
	codegen_binary_float_op(func_name, varyings, vector_width,
	                        [&](std::vector<std::string> varyings, int vector_width) {
		                        std::string s;
		                        if (vector_width == 1)
			                        s = join(varyings[0], " float ret = { ", func_name, "(a.x, b.x)");
		                        else
			                        s = join(varyings[0], " float", vector_width, " ret = { ", func_name, "(a.x, b.x)");
		                        if (vector_width > 1)
			                        s += join(", ", func_name, "(a.y, b.y)");
		                        if (vector_width > 2)
			                        s += join(", ", func_name, "(a.z, b.z)");
		                        if (vector_width > 3)
			                        s += join(", ", func_name, "(a.w, b.w)");
		                        s += " }; return ret;";
		                        statement(s);
	                        });
};

void CompilerISPC::codegen_binary_float_op_scalar_return(
    std::string func_name, std::vector<std::vector<std::string>> &varyings, std::vector<uint32_t> &vector_width,
    const std::function<void(std::vector<std::string> varyings, uint32_t vector_width)> &func)
{
	statement("//////////////////////////////");
	statement("// ", func_name);
	statement("//////////////////////////////");

	for (auto &w : vector_width)
	{
		for (auto &v : varyings)
		{
			if (w == 1)
				statement("SPIRV_INLINE ", v[0], " float ", func_name, "(", v[1], " float a, ", v[2], " float b)");
			else
				statement("SPIRV_INLINE ", v[0], " float ", func_name, "(", v[1], " float", w, "& a, ", v[2], " float",
				          w, "& b)");

			begin_scope();
			func(v, w);
			end_scope();
			statement("");
		}
	}
};

// varyings/vector widths are : return, arg1, arg2
void CompilerISPC::codegen_unary_float_op(
    std::string func_name, std::vector<std::vector<std::string>> &varyings, std::vector<uint32_t> &vector_width,
    const std::function<void(std::vector<std::string> varyings, uint32_t vector_width)> &func)
{
	statement("//////////////////////////////");
	statement("// ", func_name);
	statement("//////////////////////////////");

	for (auto &w : vector_width)
	{
		for (auto &v : varyings)
		{
			if (w == 1)
				statement("SPIRV_INLINE ", v[0], " float ", func_name, "(", v[1], " float a)");
			else
				statement("SPIRV_INLINE ", v[0], " float", w, " ", func_name, "(", v[1], " float", w, "& a)");

			begin_scope();
			func(v, w);
			end_scope();
			statement("");
		}
	}
};

// varyings/vector widths are : return, arg1, arg2
void CompilerISPC::codegen_unary_float_op_scalar_return(
    std::string func_name, std::vector<std::vector<std::string>> &varyings, std::vector<uint32_t> &vector_width,
    const std::function<void(std::vector<std::string> varyings, uint32_t vector_width)> &func)
{
	statement("//////////////////////////////");
	statement("// ", func_name);
	statement("//////////////////////////////");

	for (auto &w : vector_width)
	{
		for (auto &v : varyings)
		{
			if (w == 1)
				statement("SPIRV_INLINE ", v[0], " float ", func_name, "(", v[1], " float a)");
			else
				statement("SPIRV_INLINE ", v[0], " float ", func_name, "(", v[1], " float", w, "& a)");

			begin_scope();
			func(v, w);
			end_scope();
			statement("");
		}
	}
};

// varyings/vector widths are : return, arg1
void CompilerISPC::codegen_unary_float_op_simple(std::string func_name, std::vector<std::vector<std::string>> &varyings,
                                                 std::vector<uint32_t> &vector_width)
{
	codegen_unary_float_op(func_name, varyings, vector_width, [&](std::vector<std::string> varyings, int vector_width) {
		std::string s;
		if (vector_width == 1)
			s = join(varyings[0], " float ret = { ", func_name, "(a.x)");
		else
			s = join(varyings[0], " float", vector_width, " ret = { ", func_name, "(a.x)");
		if (vector_width > 1)
			s += join(", ", func_name, "(a.y)");
		if (vector_width > 2)
			s += join(", ", func_name, "(a.z)");
		if (vector_width > 3)
			s += join(", ", func_name, "(a.w)");
		s += " }; return ret;";
		statement(s);
	});
};

// varyings/vector widths are : return, arg1, arg2
void CompilerISPC::codegen_unary_float_op_scalar_bool_return(
    std::string func_name, std::vector<std::vector<std::string>> &varyings, std::vector<uint32_t> &vector_width,
    const std::function<void(std::vector<std::string> varyings, uint32_t vector_width)> &func)
{
	statement("//////////////////////////////");
	statement("// ", func_name);
	statement("//////////////////////////////");

	for (auto &w : vector_width)
	{
		for (auto &v : varyings)
		{
			if (w == 1)
				statement("SPIRV_INLINE ", v[0], " bool ", func_name, "(", v[1], " float a)");
			else
				statement("SPIRV_INLINE ", v[0], " bool ", func_name, "(", v[1], " float", w, "& a)");

			begin_scope();
			func(v, w);
			end_scope();
			statement("");
		}
	}
};

// varyings/vector widths are : return, arg1
void CompilerISPC::codegen_unary_float_op_int_return(
    std::string func_name, std::vector<std::vector<std::string>> &varyings, std::vector<uint32_t> &vector_width,
    const std::function<void(std::vector<std::string> varyings, uint32_t vector_width)> &func)
{
	statement("//////////////////////////////");
	statement("// ", func_name);
	statement("//////////////////////////////");

	for (auto &w : vector_width)
	{
		for (auto &v : varyings)
		{
			if (w == 1)
				statement("SPIRV_INLINE ", v[0], " int ", func_name, "(", v[1], " float a)");
			else
				statement("SPIRV_INLINE ", v[0], " int", w, " ", func_name, "(", v[1], " float", w, "& a)");

			begin_scope();
			func(v, w);
			end_scope();
			statement("");
		}
	}
};

// varyings/vector widths are : return, arg1, arg2
void CompilerISPC::codegen_unary_op_scalar_return(
    std::string func_name, std::vector<std::vector<std::string>> &varyings, std::vector<std::string> &types,
    std::vector<uint32_t> &vector_width,
    const std::function<void(std::vector<std::string> varyings, std::vector<std::string> types, uint32_t vector_width)>
        &func)
{
	statement("//////////////////////////////");
	statement("// ", func_name);
	statement("//////////////////////////////");

	for (auto &w : vector_width)
	{
		for (auto &v : varyings)
		{
			if (w == 1)
				statement("SPIRV_INLINE ", v[0], " ", types[0], " ", func_name, "(", v[1], " ", types[1], " a)");
			else
				statement("SPIRV_INLINE ", v[0], " ", types[0], " ", func_name, "(", v[1], " ", types[1], w, "& a)");

			begin_scope();
			func(v, types, w);
			end_scope();
			statement("");
		}
	}
};

// varyings/vector widths are : return, arg1, arg2
void CompilerISPC::codegen_binary_op_scalar_return(
    std::string func_name, std::vector<std::vector<std::string>> &varyings, std::vector<std::string> &types,
    std::vector<uint32_t> &vector_width,
    const std::function<void(std::vector<std::string> varyings, std::vector<std::string> types, uint32_t vector_width)>
        &func)
{
	statement("//////////////////////////////");
	statement("// ", func_name);
	statement("//////////////////////////////");

	for (auto &w : vector_width)
	{
		for (auto &v : varyings)
		{
			if (w == 1)
				statement("SPIRV_INLINE ", v[0], " ", types[0], " ", func_name, "(", v[1], " ", types[1], " a, ", v[2],
				          " ", types[2], " b)");
			else
				statement("SPIRV_INLINE ", v[0], " ", types[0], " ", func_name, "(", v[1], " ", types[1], w, "& a, ",
				          v[2], " ", types[2], w, "& b)");

			begin_scope();
			func(v, types, w);
			end_scope();
			statement("");
		}
	}
};

void CompilerISPC::codegen_binary_op(
    std::string func_name, std::vector<std::vector<std::string>> &varyings, std::vector<std::string> &types,
    std::vector<uint32_t> &vector_width,
    const std::function<void(std::vector<std::string> varyings, std::vector<std::string> types, uint32_t vector_width)>
        &func)
{
	statement("//////////////////////////////");
	statement("// ", func_name);
	statement("//////////////////////////////");

	for (auto &w : vector_width)
	{
		for (auto &v : varyings)
		{
			if (w == 1)
				statement("SPIRV_INLINE ", v[0], " ", types[0], " ", func_name, "(", v[1], " ", types[1], " a, ", v[2],
				          " ", types[2], " b)");
			else
				statement("SPIRV_INLINE ", v[0], " ", types[0], w, " ", func_name, "(", v[1], " ", types[1], w, "& a, ",
				          v[2], " ", types[2], w, "& b)");

			begin_scope();
			func(v, types, w);
			end_scope();
			statement("");
		}
	}
};

void CompilerISPC::emit_stdlib()
{
	auto &execution = get_entry_point();

	statement("//////////////////////////////");
	statement("// This ISPC spirv stdlib kernel is autogenerated by spirv-cross.");
	statement("//////////////////////////////");

	if (debug)
		statement("#define SPIRV_INLINE");
	else
		statement("#define SPIRV_INLINE inline");

	statement("");
	statement("//////////////////////////////");
	statement("// Default Types");
	statement("//////////////////////////////");
	statement("typedef float mat3[3][3];");
	statement("typedef float mat4[4][4];");

	for (const string &t : std::vector<string>{ "float", "int", "bool" })
	{
		for (const uint32_t &w : std::vector<uint32_t>{ 1, 2, 3, 4 })
		{
			codegen_default_structs(t, w);
		}
	}
	statement("");

	statement("//////////////////////////////");
	statement("// Default Image Types");
	statement("//////////////////////////////");
	codegen_default_pixel_structs(4);
	codegen_default_image_structs(2);

	// These provide us with some simple codegen to cast uniforms to varyings.
	// Is a no-op when casting a varying to a varying.
	statement("//////////////////////////////");
	statement("// Default Varying Casts");
	statement("//////////////////////////////");
	for (const string &t : std::vector<string>{ "float", "int" }) //"bool" There seems to be an issue with bools
	{
		for (const string &v : std::vector<string>{ "uniform", "varying" })
		{
			for (const uint32_t &w : std::vector<uint32_t>{ 1, 2, 3, 4 })
			{
				std::string op = join("inline varying ", t);
				if (w > 1)
					op += join(w);
				op += join(" to_varying(");
				op += join(v, " ", t);
				if (w > 1)
					op += join(w, "&");
				op += join(" a) { return (varying ", t);
				if (w > 1)
					op += join(w);
				op += ")a; }";
				statement(op);
			}
		}
	}
	statement("");

	statement("//////////////////////////////");
	statement("// Default Constructors");
	statement("//////////////////////////////");
	for (const string &t : std::vector<string>{ "float", "int", "bool" })
	{
		for (const bool &v : std::vector<bool>{ true, false })
		{
			for (const uint32_t &w : std::vector<uint32_t>{ 2, 3, 4 })
			{
				uint32_t arg_widths[][4] = {
					{ w, 1, 1, 1 }, // all
					{ 1, 1, 1, 1 }, // all
					{ 2, 1, 1, 1 }, // float 3 and larger
					{ 3, 1, 1, 1 }, // float 4 and larger
				};

				codegen_constructor(t, v, w, 1, arg_widths[0]);
				codegen_constructor(t, v, w, 1, arg_widths[1]);
				codegen_constructor(t, v, w, w, arg_widths[1]);

				if (w > 2)
				{
					codegen_constructor(t, v, w, w - 1, arg_widths[2]);
				}
				if (w > 3)
				{
					codegen_constructor(t, v, w, w - 2, arg_widths[3]);
				}
			}
			statement("");
		}
	}

	statement("");
	statement("//////////////////////////////");
	statement("// Default Operators");
	statement("//////////////////////////////");

	//    { "*", "/", "%", "+", "-" };
	for (const string &t : std::vector<string>{ "float", "int" })
	{
		for (const uint32_t &w : std::vector<uint32_t>{ 2, 3, 4 })
		{
			//            codegen_load_op(t, w);
			//            codegen_store_op(t, w);
			for (const string &bop : std::vector<string>{ "*", "/", "+", "-" })
			{
				codegen_default_binary_op(t, w, bop);
			}
		}
	}

	//
	// Ternary Op
	//
	{
		vector<vector<string>> default_varying = {
			{ "varying", "varying", "varying", "varying" },
			{ "uniform", "uniform", "uniform", "uniform" },
		};
		vector<vector<string>> complex_varying = {
			{ "varying", "varying", "varying", "varying" },
			{ "varying", "uniform", "varying", "varying" },
			{ "varying", "varying", "uniform", "varying" },
			{ "uniform", "uniform", "uniform", "uniform" },
		};
		vector<vector<string>> mixed_varying = {
			{ "varying", "varying", "uniform", "uniform" },
			{ "uniform", "uniform", "uniform", "uniform" },
		};

		// NOTE the uniform args for clamp
		codegen_ternary_float_op_simple("clamp", mixed_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_ternary_float_op(
		    "mix", complex_varying, vector<uint32_t>{ 1, 2, 3, 4 },
		    [&](std::vector<std::string> varyings, int vector_width) { statement("return a + c * (b - a);"); });
		codegen_ternary_float_op(
		    "fma", default_varying, vector<uint32_t>{ 1, 2, 3, 4 },
		    [&](std::vector<std::string> varyings, int vector_width) { statement("return a * b + c;"); });
		codegen_ternary_float_op("smoothstep", default_varying, vector<uint32_t>{ 1 },
		                         [&](std::vector<std::string> varyings, int vector_width) {
			                         statement(varyings[0], " float t = clamp((c - a) / (b - a), 0.0f, 1.0f);");
			                         statement("return t * t * (3.0f - 2.0f * t);");
		                         });
		codegen_ternary_float_op_simple("smoothstep", default_varying, vector<uint32_t>{ 2, 3, 4 });
#if 0
        // Refract has different argument widths, as the last argument is scalar
        // This is causing an issue in ISPC - not investigated yet.
        vector<vector<uint32_t>> widths = { { 2, 2, 2, 1 },{ 3, 3, 3, 1 },{ 4, 4, 4, 1 } };
        codegen_ternary_float_op_multiple_widths("refract", default_varying, widths, [&](std::vector<std::string> varyings, std::vector<uint32_t> vector_widths)
        {
            statement(varyings[0], " float", vector_widths[0], " ret;");
            statement(varyings[0], " float k = 1.0f - c * c * (1.0f - dot(b, a) * dot(b, a));");
            statement("if (k < 0.0f) ret = float", vector_widths[0], "_init(0.0f);");
            statement("else ret = float", vector_widths[0], "_init(c) * a - float", vector_widths[0], "_init(c * dot(b, a) + sqrt(k)) * b;");
            statement("return ret;");
        });
#endif
	}

	//
	// Binary Op
	//
	{
		vector<vector<string>> default_varying = {
			{ "varying", "varying", "varying" },
			{ "varying", "varying", "uniform" },
			{ "varying", "uniform", "varying" },
			{ "uniform", "uniform", "uniform" },
		};

		vector<vector<string>> mixed_varying = {
			{ "varying", "varying", "uniform" },
			{ "varying", "uniform", "varying" },
		};

		codegen_binary_float_op_scalar_return("dot", default_varying, vector<uint32_t>{ 2, 3, 4 },
		                                      [&](std::vector<std::string> varyings, int vector_width) {
			                                      switch (vector_width)
			                                      {
			                                      case 4:
				                                      statement(
				                                          "return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;");
				                                      break;
			                                      case 3:
				                                      statement("return a.x * b.x + a.y * b.y + a.z * b.z;");
				                                      break;
			                                      case 2:
				                                      statement("return a.x * b.x + a.y * b.y;");
				                                      break;
			                                      }
		                                      });

		codegen_binary_float_op("reflect", default_varying, vector<uint32_t>{ 2, 3, 4 },
		                        [&](std::vector<std::string> varyings, int vector_width) {
			                        statement("return a - b * (dot(a, b) * 2.0f);");
		                        });

		// all varying or all uniform versions already exist
		codegen_binary_float_op("min", mixed_varying, vector<uint32_t>{ 1 },
		                        [&](std::vector<std::string> varyings, int vector_width) {
			                        // uniforms need promoting to varying for the select to work and any implicit conversions
			                        if (varyings[0] == "varying")
			                        {
				                        statement("return min(to_varying(a), to_varying(b));");
			                        }
			                        else
			                        {
				                        statement("return min(a, b)");
			                        }
		                        });
		codegen_binary_float_op_simple("min", default_varying, vector<uint32_t>{ 2, 3, 4 });

		codegen_binary_float_op("max", mixed_varying, vector<uint32_t>{ 1 },
		                        [&](std::vector<std::string> varyings, int vector_width) {
			                        // uniforms need promoting to varying for the select to work and any implicit conversions
			                        if (varyings[0] == "varying")
			                        {
				                        statement("return max(to_varying(a), to_varying(b));");
			                        }
			                        else
			                        {
				                        statement("return max(a, b)");
			                        }
		                        });
		codegen_binary_float_op_simple("max", default_varying, vector<uint32_t>{ 2, 3, 4 });

		codegen_binary_float_op("step", default_varying, vector<uint32_t>{ 1 },
		                        [&](std::vector<std::string> varyings, int vector_width) {
			                        // uniforms need promoting to varying for the select to work and any implicit conversions
			                        if (varyings[0] == "varying")
			                        {
				                        statement("return to_varying(b) >= to_varying(a) ? 1.0f : 0.0f;");
			                        }
			                        else
			                        {
				                        statement("return b >= a ? 1.0f : 0.0f;");
			                        }
		                        });
		codegen_binary_float_op_simple("step", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_binary_float_op_simple("pow", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_binary_float_op("mod", default_varying, vector<uint32_t>{ 1 },
		                        [&](std::vector<std::string> varyings, int vector_width) {
			                        // uniforms need promoting to varying for the select to work and any implicit conversions
			                        if (varyings[0] == "varying")
			                        {
				                        statement("varying float va = a;");
				                        statement("varying float vb = b;");
				                        statement("return va - vb * floor(va / vb);");
			                        }
			                        else
			                        {
				                        statement("return a - b * floor(a / b);");
			                        }
		                        });
		codegen_binary_float_op_simple("mod", default_varying, vector<uint32_t>{ 2, 3, 4 });

		// Return a bool
		codegen_binary_op("notEqual", default_varying, vector<string>{ "bool", "float", "float" },
		                  vector<uint32_t>{ 1, 2, 3, 4 },
		                  [&](std::vector<std::string> varyings, std::vector<std::string> types, int vector_width) {
			                  switch (vector_width)
			                  {
			                  case 1:
				                  statement("return a != b;");
				                  break;
			                  case 2:
				                  statement(varyings[0], " ", types[0], vector_width,
				                            " ret = { a.x != b.x, a.y != b.y }; return ret;");
				                  break;
			                  case 3:
				                  statement(varyings[0], " ", types[0], vector_width,
				                            " ret = { a.x != b.x, a.y != b.y, a.z != b.z }; return ret;");
				                  break;
			                  case 4:
				                  statement(varyings[0], " ", types[0], vector_width,
				                            " ret = { a.x != b.x, a.y != b.y, a.z != b.z, a.w != b.w }; return ret;");
				                  break;
			                  }
		                  });

		// float3 version only
		codegen_binary_float_op("cross", default_varying, vector<uint32_t>{ 3 },
		                        [&](std::vector<std::string> varyings, int vector_width) {
			                        statement(varyings[0], " float", vector_width, " ret;");
			                        statement("ret.x = (a.y * b.z) - (a.z * b.y);");
			                        statement("ret.y = (a.z * b.x) - (a.x * b.z);");
			                        statement("ret.z = (a.x * b.y) - (a.y * b.x);");
			                        statement("return ret;");
		                        });
	}

	//
	// Unnary Op
	//
	{
		vector<vector<string>> default_varying = {
			{ "varying", "varying" },
			{ "uniform", "uniform" },
		};

		codegen_unary_float_op_scalar_return(
		    "length", default_varying, vector<uint32_t>{ 2, 3, 4 },
		    [&](std::vector<std::string> varyings, int vector_width) { statement("return sqrt(dot(a, a));"); });

		codegen_unary_float_op_simple("abs", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op_simple("acos", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op_simple("asin", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op_simple("atan", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op_simple("cos", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op_simple("sin", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op_simple("tan", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op_simple("floor", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op_simple("round", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op_simple("ceil", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op_simple("log", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op(
		    "log2", default_varying, vector<uint32_t>{ 1 },
		    [&](std::vector<std::string> varyings, int vector_width) { statement("return log(a) / log(2.0f);"); });
		codegen_unary_float_op_simple("log2", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op_simple("rcp", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op_simple("sqrt", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op_simple("rsqrt", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op_simple("exp", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op("cosh", default_varying, vector<uint32_t>{ 1 },
		                       [&](std::vector<std::string> varyings, int vector_width) {
			                       statement("return (exp(a) + exp(-a)) / 2.0f;");
		                       });
		codegen_unary_float_op_simple("cosh", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op("sinh", default_varying, vector<uint32_t>{ 1 },
		                       [&](std::vector<std::string> varyings, int vector_width) {
			                       statement("return (exp(a) - exp(-a)) / 2.0f;");
		                       });
		codegen_unary_float_op_simple("sinh", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op(
		    "tanh", default_varying, vector<uint32_t>{ 1 },
		    [&](std::vector<std::string> varyings, int vector_width) { statement("return sinh(a) / cosh(a);"); });
		codegen_unary_float_op_simple("tanh", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op(
		    "degrees", default_varying, vector<uint32_t>{ 1 },
		    [&](std::vector<std::string> varyings, int vector_width) { statement("return (180.0f * a) / PI;"); });
		codegen_unary_float_op_simple("degrees", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op(
		    "radians", default_varying, vector<uint32_t>{ 1 },
		    [&](std::vector<std::string> varyings, int vector_width) { statement("return (PI * a) / 180.0f;"); });
		codegen_unary_float_op_simple("radians", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op(
		    "fract", default_varying, vector<uint32_t>{ 1, 2, 3, 4 },
		    [&](std::vector<std::string> varyings, int vector_width) { statement("return a - floor(a);"); });
		codegen_unary_float_op(
		    "normalize", default_varying, vector<uint32_t>{ 2, 3, 4 },
		    [&](std::vector<std::string> varyings, int vector_width) { statement("return a / length(a);"); });
		codegen_unary_float_op(
		    "sign", default_varying, vector<uint32_t>{ 1 },
		    [&](std::vector<std::string> varyings, int vector_width) { statement("return (a < 0.0f) ? -1 : 1;"); });
		codegen_unary_float_op_simple("sign", default_varying, vector<uint32_t>{ 2, 3, 4 });
		codegen_unary_float_op(
		    "exp2", default_varying, vector<uint32_t>{ 1 },
		    [&](std::vector<std::string> varyings, int vector_width) { statement("return pow(2.0f, a);"); });
		codegen_unary_float_op_simple("exp2", default_varying, vector<uint32_t>{ 2, 3, 4 });

		// Int return type
		codegen_unary_float_op_int_return(
		    "trunc", vector<vector<string>>{ { "varying", "varying" } }, vector<uint32_t>{ 1, 2, 3, 4 },
		    [&](std::vector<std::string> varyings, int vector_width) {
			    switch (vector_width)
			    {
			    case 1:
				    statement("return (int)floor(a);");
				    break;
			    case 2:
				    statement(varyings[0], " int", vector_width, " ret = { trunc(a.x), trunc(a.y) }; return ret;");
				    break;
			    case 3:
				    statement(varyings[0], " int", vector_width,
				              " ret = { trunc(a.x), trunc(a.y), trunc(a.z) }; return ret;");
				    break;
			    case 4:
				    statement(varyings[0], " int", vector_width,
				              " ret = { trunc(a.x), trunc(a.y), trunc(a.z), trunc(a.w)}; return ret;");
				    break;
			    }
		    });

		// Scalar return
		codegen_unary_op_scalar_return(
		    "all", vector<vector<string>>{ { "varying", "varying" } }, vector<string>{ "bool", "bool" },
		    vector<uint32_t>{ 2, 3, 4 },
		    [&](std::vector<std::string> varyings, std::vector<std::string> types, int vector_width) {
			    switch (vector_width)
			    {
			    case 2:
				    statement("return all(a.x) && all(a.y);");
				    break;
			    case 3:
				    statement("return all(a.x) && all(a.y) && all(a.z);");
				    break;
			    case 4:
				    statement("return all(a.x) && all(a.y) && all(a.z) && all(a.w);");
				    break;
			    }
		    });

		codegen_unary_op_scalar_return(
		    "any", vector<vector<string>>{ { "varying", "varying" } }, vector<string>{ "bool", "bool" },
		    vector<uint32_t>{ 2, 3, 4 },
		    [&](std::vector<std::string> varyings, std::vector<std::string> types, int vector_width) {
			    switch (vector_width)
			    {
			    case 2:
				    statement("return any(a.x) || any(a.y);");
				    break;
			    case 3:
				    statement("return any(a.x) || any(a.y) || any(a.z);");
				    break;
			    case 4:
				    statement("return any(a.x) || any(a.y) || any(a.z) || any(a.w);");
				    break;
			    }
		    });
	}

	//
	// Binary Op
	//
	{
		vector<vector<string>> default_varying = {
			{ "varying", "varying", "varying" },
			{ "varying", "varying", "uniform" },
			{ "varying", "uniform", "varying" },
			{ "uniform", "uniform", "uniform" },
		};

		// distance depends on length, so must be after it
		codegen_binary_float_op_scalar_return("distance", default_varying, vector<uint32_t>{ 1, 2, 3, 4 },
		                                      [&](std::vector<std::string> varyings, int vector_width) {
			                                      switch (vector_width)
			                                      {
			                                      case 1:
				                                      statement("return abs(a - b);");
				                                      break;
			                                      default:
				                                      statement("return length(a - b);");
				                                      break;
			                                      }
		                                      });
	}

	// Atomics
	// Currently implemented assumed atomic buffer counters. Probably needs work for non buffer based atomics
	{
		statement("");
		statement("//////////////////////////////");
		statement("// Atomics");
		statement("//////////////////////////////");
		vector<string> op = { "atomic_add", "atomic_subtract", "atomic_min", "atomic_max",
			                  "atomic_and", "atomic_or",       "atomic_xor", "atomic_swap" };
		for (auto &o : op)
		{
			statement("SPIRV_INLINE varying int ", o, "(uniform int * uniform ptr, varying int value)");
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

			statement("SPIRV_INLINE varying int ", o, "(uniform int * uniform ptr, uniform int value)");
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

	// Images
	// Currently implemented assumed atomic buffer counters. Probably needs work for non buffer based atomics
	{
		statement("");
		statement("//////////////////////////////");
		statement("// Image Load/Store");
		statement("//////////////////////////////");
		statement("const varying float r_255 = 1.0 / 255.0;");
		statement("");
		statement("SPIRV_INLINE varying float4 imageLoad(uniform image2D &image, varying int2 coord)");
		begin_scope();
		statement("varying float4 res = float4_init(0.0);");
		statement("if (coord.x >= image.width || coord.y >= image.height)");
		statement("    return res;");
		statement("varying unsigned int index = coord.y * image.width + coord.x;");
		statement("varying pixel4D pix = image.data[index];");
		statement("res.x = ((float)pix.r) * r_255;");
		statement("res.y = ((float)pix.g) * r_255;");
		statement("res.z = ((float)pix.b) * r_255;");
		statement("res.w = ((float)pix.a) * r_255;");
		statement("return res;");
		end_scope();
		statement("");

		statement("");
		statement("SPIRV_INLINE void imageStore(uniform image2D &image, varying int2 coord, varying float4 rgba)");
		begin_scope();
		statement("if (coord.x >= image.width || coord.y >= image.height)");
		statement("    return;");
		statement("varying unsigned int index = coord.y * image.width + coord.x;");
		statement("varying pixel4D pix;");
		statement("pix.r = (unsigned int8)(rgba.x * 255);");
		statement("pix.g = (unsigned int8)(rgba.y * 255);");
		statement("pix.b = (unsigned int8)(rgba.z * 255);");
		statement("pix.a = (unsigned int8)(rgba.w * 255);");
		statement("image.data[index] = pix;");
		end_scope();
		statement("");
	}
}