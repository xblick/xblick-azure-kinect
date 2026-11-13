// xblick-azure-kinect-parameters: a node's parameters over its ROS 2 parameter services (Durchblick's
// xblick/parameters.hpp), as `ros2 param`. Node-agnostic (any node serving them); Durchblick's `xblick param`
// does the same in Python, this one for a camera PC with only the C++ part.
//
//   xblick-azure-kinect-parameters --node robot/azurekinect --list
//   xblick-azure-kinect-parameters --node robot/azurekinect --describe [NAME ...]   (none: all)
//   xblick-azure-kinect-parameters --node robot/azurekinect --get [NAME ...]        (none: all)
//   xblick-azure-kinect-parameters --node robot/azurekinect --set NAME:=VALUE ... [--atomically]
//
// VALUE as a YAML scalar (true, 15625, 0.5, NFOV_UNBINNED), read for the parameter's type. Zenoh: as the camera node (--connect, --listen,
// --zenoh-config). Exits 1 if a set fails.
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "args.hpp"
#include "xblick/camera.hpp"
#include "xblick/parameters.hpp"

namespace
{
    using namespace xblick;
    using xblick::k4a_node::Args;

    std::string describe_line(const ParameterDescriptor& d)
    {
        std::string line = d.name + "  " + parameter_type::name(d.type) + (d.read_only ? "  read-only (restart)" : "  runtime");
        for (const IntegerRange& r : d.integer_range)
        {
            line += "  [" + std::to_string(r.from_value) + ", " + std::to_string(r.to_value) + "]" +
                    (r.step > 1 ? " step " + std::to_string(r.step) : "");
        }
        for (const FloatingPointRange& r : d.floating_point_range)
        {
            line += "  [" + std::to_string(r.from_value) + ", " + std::to_string(r.to_value) + "]";
        }
        if (not d.additional_constraints.empty())
        {
            line += "  " + d.additional_constraints;
        }
        return line + "\n    " + d.description;
    }
}  // namespace

int main(int argc, char** argv)
{
    try
    {
        Args args(argc, argv);
        const std::string node = args.get("node", "");
        if (node.empty())
        {
            throw std::invalid_argument("--node <prefix>, e.g. robot/azurekinect");
        }
        SessionOptions session_options{.connect = args.list("connect"), .listen = args.list("listen")};
        session_options.config_file = args.get("zenoh-config", "");
        const bool list = args.flag("list"), atomically = args.flag("atomically");
        const bool describing = args.given("describe"), getting = args.given("get");
        const auto describe = args.list("describe");
        const auto get = args.list("get");
        const auto set = args.list("set");
        args.check_all_used();
        if (int(list) + int(describing) + int(getting) + int(not set.empty()) != 1)
        {
            throw std::invalid_argument("one of --list, --describe, --get, --set");
        }

        zenoh::Session session = open_session(session_options);
        ParameterClient client(session, node);
        const auto all = [&]() { return client.list().names; };
        if (list)
        {
            for (const std::string& name : all())
            {
                std::cout << name << "\n";
            }
        }
        else if (describing)
        {
            for (const ParameterDescriptor& d : client.describe(describe.empty() ? all() : describe).descriptors)
            {
                std::cout << describe_line(d) << "\n";
            }
        }
        else if (getting)
        {
            const std::vector<std::string> names = get.empty() ? all() : get;
            const auto values = client.get(names).values;
            for (std::size_t i = 0; i < names.size(); ++i)
            {
                std::cout << names[i] << ": " << parameter_text(values[i]) << "\n";
            }
        }
        else
        {
            std::vector<std::string> names, texts;
            for (const std::string& assignment : set)
            {
                const std::size_t at = assignment.find(":=");
                if (at == std::string::npos)
                {
                    throw std::invalid_argument("--set " + assignment + ": NAME:=VALUE expected");
                }
                names.push_back(assignment.substr(0, at));
                texts.push_back(assignment.substr(at + 2));
            }
            // each value read for its parameter's type (as --param at the start)
            const auto descriptors = client.describe(names).descriptors;
            std::vector<Parameter> parameters;
            for (std::size_t i = 0; i < names.size(); ++i)
            {
                parameters.push_back({names[i], parameter_from_text(descriptors.at(i).type, texts[i])});
            }
            bool ok = true;
            if (atomically)
            {
                const SetParametersResult r = client.set_atomically(parameters).result;
                std::cout << (r.successful ? "set" : "refused: " + r.reason) << "\n";
                ok = r.successful;
            }
            else
            {
                const auto results = client.set(parameters).results;
                for (std::size_t i = 0; i < parameters.size(); ++i)
                {
                    std::cout << parameters[i].name << ": "
                              << (results[i].successful ? "set" : "refused: " + results[i].reason) << "\n";
                    ok = ok and results[i].successful;
                }
            }
            return ok ? 0 : 1;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "xblick-azure-kinect-parameters: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
