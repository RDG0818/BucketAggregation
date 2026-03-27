#pragma once

namespace search::algorithms
{
    // A* is fully implemented by the default algorithm traits.
    // This struct just gives a convenient way of declaring that that's what we want.
    template<typename Domain>
    struct a
    {
        using domain_type = Domain;
    };
}