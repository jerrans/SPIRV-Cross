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
//    resource_entry_arguments_init.push_back(join(instance_name, " = a_", instance_name, ";"));

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

	if (type.basetype == SPIRType::Image || type.basetype == SPIRType::SampledImage ||
	    type.basetype == SPIRType::AtomicCounter)
	{
		statement("internal::Resource<", type_name, type_to_array_glsl(type), "> ", instance_name, "__;");
		statement_no_indent("#define ", instance_name, " __res->", instance_name, "__.get()");
		resource_registrations.push_back(
		    join("s.register_resource(", instance_name, "__", ", ", descriptor_set, ", ", binding, ");"));
	}
	else
	{
		statement("internal::UniformConstant<", type_name, type_to_array_glsl(type), "> ", instance_name, "__;");
		statement_no_indent("#define ", instance_name, " __res->", instance_name, "__.get()");
		resource_registrations.push_back(
		    join("s.register_uniform_constant(", instance_name, "__", ", ", location, ");"));
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
#if 0
	if (emitted)
		statement("");

	statement("inline void init(spirv_cross_shader& s)");
	begin_scope();
	statement(resource_type, "::init(s);");
	for (auto &reg : resource_registrations)
		statement(reg);
	end_scope();
	resource_registrations.clear();

	end_scope_decl();

	statement("");
	statement("Resources* __res;");
	if (get_entry_point().model == ExecutionModelGLCompute)
		statement("ComputePrivateResources __priv_res;");
	statement("");
#endif
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

	if (emitted)
		statement("");

    // Emit the input globals
    statement("");
    statement("//////////////////////////////");
    statement("// Input Globals");
    statement("//////////////////////////////");
    for (auto &reg : resource_registrations)
        statement(reg);
    resource_registrations.clear();
    statement("");


    statement("");
    statement("//////////////////////////////");
    statement("// Work Group Variables");
    statement("//////////////////////////////");
    statement("uniform int<3> gl_WorkGroupID;");
    statement("varying int<3> gl_GlobalInvocationID;");
    statement("varying int<3> gl_LocalInvocationID;");
    statement("varying int    gl_LocalInvocationIndex;");
    statement("uniform int<3> gl_WorkGroupSize = {", execution.workgroup_size.x, ", ", execution.workgroup_size.y, ", ", execution.workgroup_size.z, "};");
    statement("");

    if (requires_op_dot)
    {
        statement("");
        statement("//////////////////////////////");
        statement("// Dot Product");
        statement("//////////////////////////////");
        statement("inline float dot(float4& lhs, float4& rhs)");
        begin_scope();
        statement("return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;");
        end_scope();
        statement("");
        statement("inline float dot(float3& lhs, float3& rhs)");
        begin_scope();
        statement("return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;");
        end_scope();
        statement("");
        statement("inline float dot(float2& lhs, float2& rhs)");
        begin_scope();
        statement("return lhs.x * rhs.x + lhs.y * rhs.y;");
        end_scope();
        statement("");
    }

    if (requires_op_len)
    {
        statement("");
        statement("//////////////////////////////");
        statement("// Length");
        statement("//////////////////////////////");
        statement("inline float length(float3& rhs)");
        begin_scope();
        statement("return sqrt(dot(rhs, rhs));");
        end_scope();
        statement("");
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
    
    if (handler.propogate_ispc_varyings_for_builtins())
    {
        printf("Propogated");
    }
}

string CompilerISPC::compile()
{
	// Force a classic "C" locale, reverts when function returns
	ClassicLocale classic_locale;

    // Convert the use of global variables to recursively-passed function parameters
//    localize_global_variables();
    extract_global_variables_from_functions();

	// Do not deal with ES-isms like precision, older extensions and such.
	options.es = false;
	options.version = 450;
	backend.float_literal_suffix = true;
	backend.double_literal_suffix = false;
	backend.long_long_literal_suffix = true;
	backend.uint32_t_literal_suffix = true;
	backend.basic_int_type = "int";
    backend.basic_uint_type = "int";
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
    statement("");
    statement("//////////////////////////////");
    statement("// ISPC Entry Points");
    statement("//////////////////////////////");

    if (resource_entry_arguments.empty())
        statement("export void ispc_main(uniform int wg[3])");
    else
    {
        statement("export void ispc_main(uniform int wg[3],");
        for (uint32_t ii = 0; ii < resource_entry_arguments.size(); ii++)
        {
            if ((ii + 1) < resource_entry_arguments.size())
                statement("\t", resource_entry_arguments[ii], ",");
            else
                statement("\t", resource_entry_arguments[ii], ")");
        }
    }
    {
        begin_scope();

        statement("// Initialise the globals");
        for (auto &reg : resource_entry_arguments_init)
            statement(reg);
        resource_registrations.clear();
        statement("");

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
                    statement("uniform int<3> wgID = {x, y, z};"); //  
                    statement("gl_WorkGroupID = wgID;"); //  
                    assert(execution.workgroup_size.y == 1);
                    assert(execution.workgroup_size.z == 1);
                    statement("");
                    statement("// Vectorise the workgroup");
                    statement("foreach(ii = 0 ... ", execution.workgroup_size.x, " )");
                    {
                        begin_scope();
                        statement("// Keep it 1-dimensional for now");
                        statement("int<3> giID = {x * gl_WorkGroupSize.x + ii, 0, 0};");
                        statement("gl_GlobalInvocationID = giID;");
                        statement("int<3> liID= {ii, 0, 0};"); // 
                        statement("gl_LocalInvocationID = liID;"); // 
                        statement("gl_LocalInvocationIndex = gl_LocalInvocationID.z * gl_WorkGroupSize.x * gl_WorkGroupSize.y + gl_LocalInvocationID.y * gl_WorkGroupSize.x + gl_LocalInvocationID.x;");
                        statement("");
                        statement("main();");

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
    statement("export void ispc_get_workgroup_size(uniform int& wg[3])");
    begin_scope();
    statement("wg[0] = gl_WorkGroupSize.x;");
    statement("wg[1] = gl_WorkGroupSize.y;");
    statement("wg[2] = gl_WorkGroupSize.z;");
    end_scope();
    statement("");

    statement("");
    if (resource_entry_arguments.empty())
        statement("export void ispc_main2(uniform int wg[3])");
    else
    {
        statement("export void ispc_main2(uniform int wg[3],");
        for (uint32_t ii = 0; ii < resource_entry_arguments.size(); ii++)
        {
            if ((ii + 1) < resource_entry_arguments.size())
                statement("\t", resource_entry_arguments[ii], ",");
            else
                statement("\t", resource_entry_arguments[ii], ")");
        }
    }
    {
        begin_scope();

        statement("// Initialise the globals");
        statement("_113 = a__113;");
        statement("_113.g_param.x = 1234;");
        statement("a__113 = _113;");
        end_scope();
        statement("");
    }



}

void CompilerISPC::emit_function_prototype(SPIRFunction &func, uint64_t)
{
	local_variable_names = resource_names;
	string decl;

	auto &type = get<SPIRType>(func.return_type);
	decl += "inline ";
	decl += type_to_glsl(type);
	decl += " ";

	if (func.self == entry_point)
	{
		decl += "main";
		processing_entry_point = true;
	}
	else
		decl += to_name(func.self);

	decl += "(";
	for (auto &arg : func.arguments)
	{
		add_local_variable_name(arg.id);

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

	auto &var = get<SPIRVariable>(arg.id);

	string base = type_to_glsl(type);
	string variable_name = to_name(var.self);
	remap_variable_type_name(type, variable_name, base);

	for (uint32_t i = 0; i < type.array.size(); i++)
		base = join("std::array<", base, ", ", to_array_size(type, i), ">");

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

	base += type_to_glsl(type);
	remap_variable_type_name(type, name, base);
	bool runtime = false;

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
			base = join("std::array<", base, ", ", to_array_size(type, i), ">");
	}
	base += ' ';
	return base + name + (runtime ? "[]" : "");
}

void CompilerISPC::create_default_constructor(std::string type, std::string vector, uint32_t width, uint32_t arg_count, uint32_t arg_width[4])
{
    std::vector<string> arg_names = { "a", "b", "c", "d" };
    std::vector<string> arg_swizzles = { ".x", ".y", ".z", ".w" };

    // Create signature
    string args = "";
    for (uint32_t ac = 0; ac < arg_count; ac++)
    {
        args += join(vector, "const ", type);   // varying const float
        if (arg_width[ac] > 1)
            args += join(arg_width[ac]);        // varying const float2
        args += "&";                            // varying const float2&
        args += join(" ", arg_names[ac]);       // varying const float2& a
        if ((ac + 1) < arg_count)
            args += ", ";
    }

    statement("inline ", vector, type, width, " ", type, width, "_init(", args, " )");
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

    statement(vector, type, width, " ret = { ", init, " }; ");
    statement("return ret;");
    end_scope();
    statement("");
}

void CompilerISPC::emit_header()
{
	auto &execution = get_entry_point();

    statement("//");
    statement("//////////////////////////////");
    statement("// This ISPC kernel is autogenerated by spirv-cross.");
    statement("//////////////////////////////");
    statement("//");
    statement("");
    statement("");
    statement("//////////////////////////////");
    statement("// Default Types");
    statement("//////////////////////////////");
    statement("typedef float<2> float2;");
    statement("typedef float<3> float3;");
    statement("typedef float<4> float4;");
    statement("");
    statement("typedef int<2> int2;");
    statement("typedef int<3> int3;");
    statement("typedef int<4> int4;");

    std::vector<uint32_t> widths = { 2, 3, 4 };
    std::vector<string> types = { "float", "int" };
    std::vector<string> vector = { "varying ", "uniform " }; // note spaces

    statement("");
    statement("//////////////////////////////");
    statement("// Default Constructors");
    statement("//////////////////////////////");
    for (const string& t : types)
    {
        for (const string& v : vector)
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
        switch (type.basetype)
        {
        case SPIRType::Boolean:
            return join("bool<", type.vecsize, ">");
        case SPIRType::Int:
            return join("int<", type.vecsize, ">");
        case SPIRType::UInt:
            return join("int<", type.vecsize, ">");
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


    default:
    {
        // what is the default opcode
        printf("unknown opcode");
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
//            if (var.storage == StorageClassInput || var.storage == StorageClassUniform ||
//                var.storage == StorageClassUniformConstant || var.storage == StorageClassPushConstant)
            if (var.storage == StorageClassUniform)
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
            string vld_name = ensure_valid_name(to_name(arg_id), "");
//            string vld_name = to_name(arg_id);
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
