// Command-line options of the programs: --name value..., --flag. An option given but never asked for is an error (a
// typo must not be ignored), as is a value for a flag or several values for a single one.
#pragma once

#include <algorithm>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace durchblick::k4a_node
{
    class Args
    {
    public:
        Args(int argc, char** argv)
        {
            std::string current;
            for (int i = 1; i < argc; ++i)
            {
                const std::string a = argv[i];
                if (a.rfind("--", 0) == 0)
                {
                    current = a.substr(2);
                    if (values_.count(current))
                    {
                        throw std::invalid_argument("--" + current + " given twice");
                    }
                    values_[current];
                }
                else if (current.empty())
                {
                    throw std::invalid_argument("unexpected argument '" + a + "'");
                }
                else
                {
                    values_[current].push_back(a);
                }
            }
        }

        bool flag(const std::string& name)
        {
            used_.push_back(name);
            if (values_.count(name) and not values_[name].empty())
            {
                throw std::invalid_argument("--" + name + " takes no value");
            }
            return values_.count(name) > 0;
        }

        // given at all, with or without values (an option that takes zero or more)
        bool given(const std::string& name)
        {
            used_.push_back(name);
            return values_.count(name) > 0;
        }

        std::vector<std::string> list(const std::string& name)
        {
            used_.push_back(name);
            return values_.count(name) ? values_[name] : std::vector<std::string>{};
        }

        std::optional<std::string> get(const std::string& name)
        {
            used_.push_back(name);
            if (not values_.count(name))
            {
                return std::nullopt;
            }
            if (values_[name].size() != 1)
            {
                throw std::invalid_argument("--" + name + " takes one value");
            }
            return values_[name][0];
        }

        std::string get(const std::string& name, const std::string& fallback) { return get(name).value_or(fallback); }

        // every option given must have been asked for (a typo is an error, not ignored)
        void check_all_used() const
        {
            for (const auto& [name, _] : values_)
            {
                if (std::find(used_.begin(), used_.end(), name) == used_.end())
                {
                    throw std::invalid_argument("unknown option --" + name);
                }
            }
        }

    private:
        std::map<std::string, std::vector<std::string>> values_;
        std::vector<std::string> used_;
    };
}  // namespace durchblick::k4a_node
