// A Bison parser, made by GNU Bison 3.0.2.

// Skeleton implementation for Bison LALR(1) parsers in C++

// Copyright (C) 2002-2013 Free Software Foundation, Inc.

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// As a special exception, you may create a larger work that contains
// part or all of the Bison parser skeleton and distribute that work
// under terms of your choice, so long as that work isn't itself a
// parser generator using the skeleton or a modified version thereof
// as a parser skeleton.  Alternatively, if you modify or redistribute
// the parser skeleton itself, you may (at your option) remove this
// special exception, which will cause the skeleton and the resulting
// Bison output files to be licensed under the GNU General Public
// License without this special exception.

// This special exception was added by the Free Software Foundation in
// version 2.2 of Bison.

// Take the name prefix into account.
#define yylex   GringoNonGroundGrammar_lex

// First part of user declarations.
#line 52 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:399


#include "gringo/input/nongroundparser.hh"
#include "gringo/input/programbuilder.hh"
#include <climits> 

#define BUILDER (lexer->builder())
#define YYLLOC_DEFAULT(Current, Rhs, N)                                \
    do {                                                               \
        if (N) {                                                       \
            (Current).beginFilename = YYRHSLOC (Rhs, 1).beginFilename; \
            (Current).beginLine     = YYRHSLOC (Rhs, 1).beginLine;     \
            (Current).beginColumn   = YYRHSLOC (Rhs, 1).beginColumn;   \
            (Current).endFilename   = YYRHSLOC (Rhs, N).endFilename;   \
            (Current).endLine       = YYRHSLOC (Rhs, N).endLine;       \
            (Current).endColumn     = YYRHSLOC (Rhs, N).endColumn;     \
        }                                                              \
        else {                                                         \
            (Current).beginFilename = YYRHSLOC (Rhs, 0).endFilename; \
            (Current).beginLine     = YYRHSLOC (Rhs, 0).endLine;     \
            (Current).beginColumn   = YYRHSLOC (Rhs, 0).endColumn;   \
            (Current).endFilename   = YYRHSLOC (Rhs, 0).endFilename;   \
            (Current).endLine       = YYRHSLOC (Rhs, 0).endLine;       \
            (Current).endColumn     = YYRHSLOC (Rhs, 0).endColumn;     \
        }                                                              \
    }                                                                  \
    while (false)

using namespace Gringo;
using namespace Gringo::Input;

int GringoNonGroundGrammar_lex(void *value, Gringo::Location* loc, NonGroundParser *lexer) {
    return lexer->lex(value, *loc);
}


#line 75 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:399

# ifndef YY_NULLPTR
#  if defined __cplusplus && 201103L <= __cplusplus
#   define YY_NULLPTR nullptr
#  else
#   define YY_NULLPTR 0
#  endif
# endif

#include "grammar.hh"

// User implementation prologue.

#line 89 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:407
// Unqualified %code blocks.
#line 89 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:408


void NonGroundGrammar::parser::error(DefaultLocation const &l, std::string const &msg) {
    lexer->parseError(l, msg);
}


#line 99 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:408


#ifndef YY_
# if defined YYENABLE_NLS && YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> // FIXME: INFRINGES ON USER NAME SPACE.
#   define YY_(msgid) dgettext ("bison-runtime", msgid)
#  endif
# endif
# ifndef YY_
#  define YY_(msgid) msgid
# endif
#endif

#define YYRHSLOC(Rhs, K) ((Rhs)[K].location)
/* YYLLOC_DEFAULT -- Set CURRENT to span from RHS[1] to RHS[N].
   If N is 0, then set CURRENT to the empty location which ends
   the previous symbol: RHS[0] (always defined).  */

# ifndef YYLLOC_DEFAULT
#  define YYLLOC_DEFAULT(Current, Rhs, N)                               \
    do                                                                  \
      if (N)                                                            \
        {                                                               \
          (Current).begin  = YYRHSLOC (Rhs, 1).begin;                   \
          (Current).end    = YYRHSLOC (Rhs, N).end;                     \
        }                                                               \
      else                                                              \
        {                                                               \
          (Current).begin = (Current).end = YYRHSLOC (Rhs, 0).end;      \
        }                                                               \
    while (/*CONSTCOND*/ false)
# endif


// Suppress unused-variable warnings by "using" E.
#define YYUSE(E) ((void) (E))

// Enable debugging if requested.
#if YYDEBUG

// A pseudo ostream that takes yydebug_ into account.
# define YYCDEBUG if (yydebug_) (*yycdebug_)

# define YY_SYMBOL_PRINT(Title, Symbol)         \
  do {                                          \
    if (yydebug_)                               \
    {                                           \
      *yycdebug_ << Title << ' ';               \
      yy_print_ (*yycdebug_, Symbol);           \
      *yycdebug_ << std::endl;                  \
    }                                           \
  } while (false)

# define YY_REDUCE_PRINT(Rule)          \
  do {                                  \
    if (yydebug_)                       \
      yy_reduce_print_ (Rule);          \
  } while (false)

# define YY_STACK_PRINT()               \
  do {                                  \
    if (yydebug_)                       \
      yystack_print_ ();                \
  } while (false)

#else // !YYDEBUG

# define YYCDEBUG if (false) std::cerr
# define YY_SYMBOL_PRINT(Title, Symbol)  YYUSE(Symbol)
# define YY_REDUCE_PRINT(Rule)           static_cast<void>(0)
# define YY_STACK_PRINT()                static_cast<void>(0)

#endif // !YYDEBUG

#define yyerrok         (yyerrstatus_ = 0)
#define yyclearin       (yyempty = true)

#define YYACCEPT        goto yyacceptlab
#define YYABORT         goto yyabortlab
#define YYERROR         goto yyerrorlab
#define YYRECOVERING()  (!!yyerrstatus_)

#line 23 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:474
namespace Gringo { namespace Input { namespace NonGroundGrammar {
#line 185 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:474

  /* Return YYSTR after stripping away unnecessary quotes and
     backslashes, so that it's suitable for yyerror.  The heuristic is
     that double-quoting is unnecessary unless the string contains an
     apostrophe, a comma, or backslash (other than backslash-backslash).
     YYSTR is taken from yytname.  */
  std::string
  parser::yytnamerr_ (const char *yystr)
  {
    if (*yystr == '"')
      {
        std::string yyr = "";
        char const *yyp = yystr;

        for (;;)
          switch (*++yyp)
            {
            case '\'':
            case ',':
              goto do_not_strip_quotes;

            case '\\':
              if (*++yyp != '\\')
                goto do_not_strip_quotes;
              // Fall through.
            default:
              yyr += *yyp;
              break;

            case '"':
              return yyr;
            }
      do_not_strip_quotes: ;
      }

    return yystr;
  }


  /// Build a parser object.
  parser::parser (Gringo::Input::NonGroundParser *lexer_yyarg)
    :
#if YYDEBUG
      yydebug_ (false),
      yycdebug_ (&std::cerr),
#endif
      lexer (lexer_yyarg)
  {}

  parser::~parser ()
  {}


  /*---------------.
  | Symbol types.  |
  `---------------*/

  inline
  parser::syntax_error::syntax_error (const location_type& l, const std::string& m)
    : std::runtime_error (m)
    , location (l)
  {}

  // basic_symbol.
  template <typename Base>
  inline
  parser::basic_symbol<Base>::basic_symbol ()
    : value ()
  {}

  template <typename Base>
  inline
  parser::basic_symbol<Base>::basic_symbol (const basic_symbol& other)
    : Base (other)
    , value ()
    , location (other.location)
  {
    value = other.value;
  }


  template <typename Base>
  inline
  parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const semantic_type& v, const location_type& l)
    : Base (t)
    , value (v)
    , location (l)
  {}


  /// Constructor for valueless symbols.
  template <typename Base>
  inline
  parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const location_type& l)
    : Base (t)
    , value ()
    , location (l)
  {}

  template <typename Base>
  inline
  parser::basic_symbol<Base>::~basic_symbol ()
  {
  }

  template <typename Base>
  inline
  void
  parser::basic_symbol<Base>::move (basic_symbol& s)
  {
    super_type::move(s);
    value = s.value;
    location = s.location;
  }

  // by_type.
  inline
  parser::by_type::by_type ()
     : type (empty)
  {}

  inline
  parser::by_type::by_type (const by_type& other)
    : type (other.type)
  {}

  inline
  parser::by_type::by_type (token_type t)
    : type (yytranslate_ (t))
  {}

  inline
  void
  parser::by_type::move (by_type& that)
  {
    type = that.type;
    that.type = empty;
  }

  inline
  int
  parser::by_type::type_get () const
  {
    return type;
  }


  // by_state.
  inline
  parser::by_state::by_state ()
    : state (empty)
  {}

  inline
  parser::by_state::by_state (const by_state& other)
    : state (other.state)
  {}

  inline
  void
  parser::by_state::move (by_state& that)
  {
    state = that.state;
    that.state = empty;
  }

  inline
  parser::by_state::by_state (state_type s)
    : state (s)
  {}

  inline
  parser::symbol_number_type
  parser::by_state::type_get () const
  {
    return state == empty ? 0 : yystos_[state];
  }

  inline
  parser::stack_symbol_type::stack_symbol_type ()
  {}


  inline
  parser::stack_symbol_type::stack_symbol_type (state_type s, symbol_type& that)
    : super_type (s, that.location)
  {
    value = that.value;
    // that is emptied.
    that.type = empty;
  }

  inline
  parser::stack_symbol_type&
  parser::stack_symbol_type::operator= (const stack_symbol_type& that)
  {
    state = that.state;
    value = that.value;
    location = that.location;
    return *this;
  }


  template <typename Base>
  inline
  void
  parser::yy_destroy_ (const char* yymsg, basic_symbol<Base>& yysym) const
  {
    if (yymsg)
      YY_SYMBOL_PRINT (yymsg, yysym);

    // User destructor.
    YYUSE (yysym.type_get ());
  }

#if YYDEBUG
  template <typename Base>
  void
  parser::yy_print_ (std::ostream& yyo,
                                     const basic_symbol<Base>& yysym) const
  {
    std::ostream& yyoutput = yyo;
    YYUSE (yyoutput);
    symbol_number_type yytype = yysym.type_get ();
    yyo << (yytype < yyntokens_ ? "token" : "nterm")
        << ' ' << yytname_[yytype] << " ("
        << yysym.location << ": ";
    YYUSE (yytype);
    yyo << ')';
  }
#endif

  inline
  void
  parser::yypush_ (const char* m, state_type s, symbol_type& sym)
  {
    stack_symbol_type t (s, sym);
    yypush_ (m, t);
  }

  inline
  void
  parser::yypush_ (const char* m, stack_symbol_type& s)
  {
    if (m)
      YY_SYMBOL_PRINT (m, s);
    yystack_.push (s);
  }

  inline
  void
  parser::yypop_ (unsigned int n)
  {
    yystack_.pop (n);
  }

#if YYDEBUG
  std::ostream&
  parser::debug_stream () const
  {
    return *yycdebug_;
  }

  void
  parser::set_debug_stream (std::ostream& o)
  {
    yycdebug_ = &o;
  }


  parser::debug_level_type
  parser::debug_level () const
  {
    return yydebug_;
  }

  void
  parser::set_debug_level (debug_level_type l)
  {
    yydebug_ = l;
  }
#endif // YYDEBUG

  inline parser::state_type
  parser::yy_lr_goto_state_ (state_type yystate, int yysym)
  {
    int yyr = yypgoto_[yysym - yyntokens_] + yystate;
    if (0 <= yyr && yyr <= yylast_ && yycheck_[yyr] == yystate)
      return yytable_[yyr];
    else
      return yydefgoto_[yysym - yyntokens_];
  }

  inline bool
  parser::yy_pact_value_is_default_ (int yyvalue)
  {
    return yyvalue == yypact_ninf_;
  }

  inline bool
  parser::yy_table_value_is_error_ (int yyvalue)
  {
    return yyvalue == yytable_ninf_;
  }

  int
  parser::parse ()
  {
    /// Whether yyla contains a lookahead.
    bool yyempty = true;

    // State.
    int yyn;
    /// Length of the RHS of the rule being reduced.
    int yylen = 0;

    // Error handling.
    int yynerrs_ = 0;
    int yyerrstatus_ = 0;

    /// The lookahead symbol.
    symbol_type yyla;

    /// The locations where the error started and ended.
    stack_symbol_type yyerror_range[3];

    /// The return value of parse ().
    int yyresult;

    // FIXME: This shoud be completely indented.  It is not yet to
    // avoid gratuitous conflicts when merging into the master branch.
    try
      {
    YYCDEBUG << "Starting parse" << std::endl;


    /* Initialize the stack.  The initial state will be set in
       yynewstate, since the latter expects the semantical and the
       location values to have been already stored, initialize these
       stacks with a primary value.  */
    yystack_.clear ();
    yypush_ (YY_NULLPTR, 0, yyla);

    // A new symbol was pushed on the stack.
  yynewstate:
    YYCDEBUG << "Entering state " << yystack_[0].state << std::endl;

    // Accept?
    if (yystack_[0].state == yyfinal_)
      goto yyacceptlab;

    goto yybackup;

    // Backup.
  yybackup:

    // Try to take a decision without lookahead.
    yyn = yypact_[yystack_[0].state];
    if (yy_pact_value_is_default_ (yyn))
      goto yydefault;

    // Read a lookahead token.
    if (yyempty)
      {
        YYCDEBUG << "Reading a token: ";
        try
          {
            yyla.type = yytranslate_ (yylex (&yyla.value, &yyla.location, lexer));
          }
        catch (const syntax_error& yyexc)
          {
            error (yyexc);
            goto yyerrlab1;
          }
        yyempty = false;
      }
    YY_SYMBOL_PRINT ("Next token is", yyla);

    /* If the proper action on seeing token YYLA.TYPE is to reduce or
       to detect an error, take that action.  */
    yyn += yyla.type_get ();
    if (yyn < 0 || yylast_ < yyn || yycheck_[yyn] != yyla.type_get ())
      goto yydefault;

    // Reduce or error.
    yyn = yytable_[yyn];
    if (yyn <= 0)
      {
        if (yy_table_value_is_error_ (yyn))
          goto yyerrlab;
        yyn = -yyn;
        goto yyreduce;
      }

    // Discard the token being shifted.
    yyempty = true;

    // Count tokens shifted since error; after three, turn off error status.
    if (yyerrstatus_)
      --yyerrstatus_;

    // Shift the lookahead token.
    yypush_ ("Shifting", yyn, yyla);
    goto yynewstate;

  /*-----------------------------------------------------------.
  | yydefault -- do the default action for the current state.  |
  `-----------------------------------------------------------*/
  yydefault:
    yyn = yydefact_[yystack_[0].state];
    if (yyn == 0)
      goto yyerrlab;
    goto yyreduce;

  /*-----------------------------.
  | yyreduce -- Do a reduction.  |
  `-----------------------------*/
  yyreduce:
    yylen = yyr2_[yyn];
    {
      stack_symbol_type yylhs;
      yylhs.state = yy_lr_goto_state_(yystack_[yylen].state, yyr1_[yyn]);
      /* If YYLEN is nonzero, implement the default value of the
         action: '$$ = $1'.  Otherwise, use the top of the stack.

         Otherwise, the following line sets YYLHS.VALUE to garbage.
         This behavior is undocumented and Bison users should not rely
         upon it.  */
      if (yylen)
        yylhs.value = yystack_[yylen - 1].value;
      else
        yylhs.value = yystack_[0].value;

      // Compute the default @$.
      {
        slice<stack_symbol_type, stack_type> slice (yystack_, yylen);
        YYLLOC_DEFAULT (yylhs.location, slice, yylen);
      }

      // Perform the reduction.
      YY_REDUCE_PRINT (yyn);
      try
        {
          switch (yyn)
            {
  case 7:
#line 303 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.uid) = (yystack_[0].value.uid); }
#line 634 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 8:
#line 311 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, BinOp::XOR, (yystack_[2].value.term), (yystack_[0].value.term)); }
#line 640 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 9:
#line 312 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, BinOp::OR, (yystack_[2].value.term), (yystack_[0].value.term)); }
#line 646 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 10:
#line 313 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, BinOp::AND, (yystack_[2].value.term), (yystack_[0].value.term)); }
#line 652 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 11:
#line 314 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, BinOp::ADD, (yystack_[2].value.term), (yystack_[0].value.term)); }
#line 658 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 12:
#line 315 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, BinOp::SUB, (yystack_[2].value.term), (yystack_[0].value.term)); }
#line 664 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 13:
#line 316 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, BinOp::MUL, (yystack_[2].value.term), (yystack_[0].value.term)); }
#line 670 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 14:
#line 317 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, BinOp::DIV, (yystack_[2].value.term), (yystack_[0].value.term)); }
#line 676 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 15:
#line 318 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, BinOp::MOD, (yystack_[2].value.term), (yystack_[0].value.term)); }
#line 682 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 16:
#line 319 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, BinOp::POW, (yystack_[2].value.term), (yystack_[0].value.term)); }
#line 688 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 17:
#line 320 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[1].location + yystack_[0].location, UnOp::NEG, (yystack_[0].value.term)); }
#line 694 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 18:
#line 321 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[1].location + yystack_[0].location, UnOp::NOT, (yystack_[0].value.term)); }
#line 700 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 19:
#line 322 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, FWString(""), (yystack_[1].value.termvecvec), false); }
#line 706 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 20:
#line 323 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[3].location + yystack_[0].location, (yystack_[3].value.uid), (yystack_[1].value.termvecvec), false); }
#line 712 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 21:
#line 324 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[4].location + yystack_[0].location, (yystack_[3].value.uid), (yystack_[1].value.termvecvec), true); }
#line 718 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 22:
#line 325 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, UnOp::ABS, (yystack_[1].value.term)); }
#line 724 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 23:
#line 326 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[0].location, Value(FWString((yystack_[0].value.uid)))); }
#line 730 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 24:
#line 327 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[0].location, Value((yystack_[0].value.num))); }
#line 736 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 25:
#line 328 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[0].location, Value(FWString((yystack_[0].value.uid)), false)); }
#line 742 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 26:
#line 329 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[0].location, Value(true)); }
#line 748 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 27:
#line 330 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[0].location, Value(false)); }
#line 754 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 28:
#line 336 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.termvec) = BUILDER.termvec(BUILDER.termvec(), (yystack_[0].value.term));  }
#line 760 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 29:
#line 337 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.termvec) = BUILDER.termvec((yystack_[2].value.termvec), (yystack_[0].value.term));  }
#line 766 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 30:
#line 341 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.termvecvec) = BUILDER.termvecvec(BUILDER.termvecvec(), (yystack_[0].value.termvec));  }
#line 772 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 31:
#line 342 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.termvecvec) = BUILDER.termvecvec();  }
#line 778 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 32:
#line 350 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, (yystack_[2].value.term), (yystack_[0].value.term)); }
#line 784 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 33:
#line 351 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, BinOp::XOR, (yystack_[2].value.term), (yystack_[0].value.term)); }
#line 790 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 34:
#line 352 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, BinOp::OR, (yystack_[2].value.term), (yystack_[0].value.term)); }
#line 796 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 35:
#line 353 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, BinOp::AND, (yystack_[2].value.term), (yystack_[0].value.term)); }
#line 802 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 36:
#line 354 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, BinOp::ADD, (yystack_[2].value.term), (yystack_[0].value.term)); }
#line 808 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 37:
#line 355 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, BinOp::SUB, (yystack_[2].value.term), (yystack_[0].value.term)); }
#line 814 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 38:
#line 356 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, BinOp::MUL, (yystack_[2].value.term), (yystack_[0].value.term)); }
#line 820 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 39:
#line 357 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, BinOp::DIV, (yystack_[2].value.term), (yystack_[0].value.term)); }
#line 826 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 40:
#line 358 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, BinOp::MOD, (yystack_[2].value.term), (yystack_[0].value.term)); }
#line 832 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 41:
#line 359 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, BinOp::POW, (yystack_[2].value.term), (yystack_[0].value.term)); }
#line 838 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 42:
#line 360 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[1].location + yystack_[0].location, UnOp::NEG, (yystack_[0].value.term)); }
#line 844 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 43:
#line 361 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[1].location + yystack_[0].location, UnOp::NOT, (yystack_[0].value.term)); }
#line 850 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 44:
#line 362 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, FWString(""), (yystack_[1].value.termvecvec), false); }
#line 856 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 45:
#line 363 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[3].location + yystack_[0].location, (yystack_[3].value.uid), (yystack_[1].value.termvecvec), false); }
#line 862 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 46:
#line 364 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[4].location + yystack_[0].location, (yystack_[3].value.uid), (yystack_[1].value.termvecvec), true); }
#line 868 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 47:
#line 365 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[2].location + yystack_[0].location, UnOp::ABS, (yystack_[1].value.termvec)); }
#line 874 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 48:
#line 366 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[0].location, Value(FWString((yystack_[0].value.uid)))); }
#line 880 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 49:
#line 367 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[0].location, Value((yystack_[0].value.num))); }
#line 886 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 50:
#line 368 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[0].location, Value(FWString((yystack_[0].value.uid)), false)); }
#line 892 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 51:
#line 369 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[0].location, Value(true)); }
#line 898 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 52:
#line 370 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[0].location, Value(false)); }
#line 904 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 53:
#line 371 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[0].location, FWString((yystack_[0].value.uid))); }
#line 910 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 54:
#line 372 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.term) = BUILDER.term(yystack_[0].location, FWString("_")); }
#line 916 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 55:
#line 378 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.termvec) = BUILDER.termvec(BUILDER.termvec(), (yystack_[0].value.term)); }
#line 922 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 56:
#line 379 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.termvec) = BUILDER.termvec((yystack_[2].value.termvec), (yystack_[0].value.term)); }
#line 928 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 57:
#line 386 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.termvec) = BUILDER.termvec(BUILDER.termvec(), (yystack_[0].value.term)); }
#line 934 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 58:
#line 387 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.termvec) = BUILDER.termvec((yystack_[2].value.termvec), (yystack_[0].value.term)); }
#line 940 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 59:
#line 391 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.termvec) = (yystack_[0].value.termvec); }
#line 946 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 60:
#line 392 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.termvec) = BUILDER.termvec(); }
#line 952 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 61:
#line 396 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.termvecvec) = BUILDER.termvecvec(BUILDER.termvecvec(), (yystack_[0].value.termvec)); }
#line 958 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 62:
#line 397 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.termvecvec) = BUILDER.termvecvec((yystack_[2].value.termvecvec), (yystack_[0].value.termvec)); }
#line 964 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 63:
#line 406 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.rel) = Relation::GT; }
#line 970 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 64:
#line 407 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.rel) = Relation::LT; }
#line 976 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 65:
#line 408 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.rel) = Relation::GEQ; }
#line 982 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 66:
#line 409 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.rel) = Relation::LEQ; }
#line 988 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 67:
#line 410 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.rel) = Relation::EQ; }
#line 994 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 68:
#line 411 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.rel) = Relation::NEQ; }
#line 1000 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 69:
#line 412 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.rel) = Relation::ASSIGN; }
#line 1006 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 70:
#line 416 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.pair) = { (yystack_[0].value.uid), BUILDER.termvecvec(BUILDER.termvecvec(), BUILDER.termvec()) << 1u }; }
#line 1012 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 71:
#line 417 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.pair) = { (yystack_[3].value.uid), (yystack_[1].value.termvecvec) << 1u }; }
#line 1018 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 72:
#line 418 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.pair) = { (yystack_[0].value.uid), BUILDER.termvecvec(BUILDER.termvecvec(), BUILDER.termvec()) << 1u | 1u }; }
#line 1024 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 73:
#line 419 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.pair) = { (yystack_[3].value.uid), (yystack_[1].value.termvecvec) << 1u | 1u }; }
#line 1030 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 74:
#line 423 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.lit) = BUILDER.boollit(yylhs.location, true); }
#line 1036 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 75:
#line 424 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.lit) = BUILDER.boollit(yylhs.location, false); }
#line 1042 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 76:
#line 425 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.lit) = BUILDER.predlit(yylhs.location, NAF::POS, (yystack_[0].value.pair).second & 1, FWString((yystack_[0].value.pair).first), TermVecVecUid((yystack_[0].value.pair).second >> 1u)); }
#line 1048 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 77:
#line 426 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.lit) = BUILDER.predlit(yylhs.location, NAF::NOT, (yystack_[0].value.pair).second & 1, FWString((yystack_[0].value.pair).first), TermVecVecUid((yystack_[0].value.pair).second >> 1u)); }
#line 1054 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 78:
#line 427 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.lit) = BUILDER.predlit(yylhs.location, NAF::NOTNOT, (yystack_[0].value.pair).second & 1, FWString((yystack_[0].value.pair).first), TermVecVecUid((yystack_[0].value.pair).second >> 1u)); }
#line 1060 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 79:
#line 428 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.lit) = BUILDER.rellit(yylhs.location, (yystack_[1].value.rel), (yystack_[2].value.term), (yystack_[0].value.term)); }
#line 1066 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 80:
#line 429 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.lit) = BUILDER.csplit((yystack_[0].value.csplit)); }
#line 1072 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 81:
#line 433 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.cspmulterm) = BUILDER.cspmulterm(yylhs.location, (yystack_[0].value.term),                     (yystack_[2].value.term)); }
#line 1078 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 82:
#line 434 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.cspmulterm) = BUILDER.cspmulterm(yylhs.location, (yystack_[3].value.term),                     (yystack_[0].value.term)); }
#line 1084 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 83:
#line 435 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.cspmulterm) = BUILDER.cspmulterm(yylhs.location, BUILDER.term(yylhs.location, Value(1)), (yystack_[0].value.term)); }
#line 1090 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 84:
#line 436 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.cspmulterm) = BUILDER.cspmulterm(yylhs.location, (yystack_[0].value.term)); }
#line 1096 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 85:
#line 440 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.cspaddterm) = BUILDER.cspaddterm(yylhs.location, (yystack_[2].value.cspaddterm), (yystack_[0].value.cspmulterm), true); }
#line 1102 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 86:
#line 441 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.cspaddterm) = BUILDER.cspaddterm(yylhs.location, (yystack_[2].value.cspaddterm), (yystack_[0].value.cspmulterm), false); }
#line 1108 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 87:
#line 442 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.cspaddterm) = BUILDER.cspaddterm(yylhs.location, (yystack_[0].value.cspmulterm)); }
#line 1114 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 88:
#line 446 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.rel) = Relation::GT; }
#line 1120 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 89:
#line 447 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.rel) = Relation::LT; }
#line 1126 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 90:
#line 448 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.rel) = Relation::GEQ; }
#line 1132 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 91:
#line 449 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.rel) = Relation::LEQ; }
#line 1138 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 92:
#line 450 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.rel) = Relation::EQ; }
#line 1144 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 93:
#line 451 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.rel) = Relation::NEQ; }
#line 1150 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 94:
#line 455 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.csplit) = BUILDER.csplit(yylhs.location, (yystack_[2].value.csplit), (yystack_[1].value.rel), (yystack_[0].value.cspaddterm)); }
#line 1156 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 95:
#line 456 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.csplit) = BUILDER.csplit(yylhs.location, (yystack_[2].value.cspaddterm),   (yystack_[1].value.rel), (yystack_[0].value.cspaddterm)); }
#line 1162 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 96:
#line 464 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.litvec) = BUILDER.litvec(BUILDER.litvec(), (yystack_[0].value.lit)); }
#line 1168 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 97:
#line 465 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.litvec) = BUILDER.litvec((yystack_[2].value.litvec), (yystack_[0].value.lit)); }
#line 1174 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 98:
#line 469 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.litvec) = (yystack_[0].value.litvec); }
#line 1180 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 99:
#line 470 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.litvec) = BUILDER.litvec(); }
#line 1186 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 100:
#line 474 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.litvec) = (yystack_[0].value.litvec); }
#line 1192 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 101:
#line 475 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.litvec) = BUILDER.litvec(); }
#line 1198 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 102:
#line 479 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.litvec) = (yystack_[0].value.litvec); }
#line 1204 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 103:
#line 480 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.litvec) = BUILDER.litvec(); }
#line 1210 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 104:
#line 484 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.fun) = AggregateFunction::SUM; }
#line 1216 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 105:
#line 485 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.fun) = AggregateFunction::SUMP; }
#line 1222 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 106:
#line 486 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.fun) = AggregateFunction::MIN; }
#line 1228 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 107:
#line 487 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.fun) = AggregateFunction::MAX; }
#line 1234 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 108:
#line 488 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.fun) = AggregateFunction::COUNT; }
#line 1240 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 109:
#line 496 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.bodyaggrelem) = { BUILDER.termvec(), (yystack_[0].value.litvec) }; }
#line 1246 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 110:
#line 497 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.bodyaggrelem) = { (yystack_[1].value.termvec), (yystack_[0].value.litvec) }; }
#line 1252 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 111:
#line 501 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.bodyaggrelemvec) = BUILDER.bodyaggrelemvec(BUILDER.bodyaggrelemvec(), (yystack_[0].value.bodyaggrelem).first, (yystack_[0].value.bodyaggrelem).second); }
#line 1258 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 112:
#line 502 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.bodyaggrelemvec) = BUILDER.bodyaggrelemvec((yystack_[2].value.bodyaggrelemvec), (yystack_[0].value.bodyaggrelem).first, (yystack_[0].value.bodyaggrelem).second); }
#line 1264 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 113:
#line 508 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.lbodyaggrelem) = { (yystack_[1].value.lit), (yystack_[0].value.litvec) }; }
#line 1270 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 114:
#line 512 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.condlitlist) = BUILDER.condlitvec(BUILDER.condlitvec(), (yystack_[0].value.lbodyaggrelem).first, (yystack_[0].value.lbodyaggrelem).second); }
#line 1276 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 115:
#line 513 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.condlitlist) = BUILDER.condlitvec((yystack_[2].value.condlitlist), (yystack_[0].value.lbodyaggrelem).first, (yystack_[0].value.lbodyaggrelem).second); }
#line 1282 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 116:
#line 519 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.aggr) = { AggregateFunction::COUNT, true, BUILDER.condlitvec() }; }
#line 1288 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 117:
#line 520 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.aggr) = { AggregateFunction::COUNT, true, (yystack_[1].value.condlitlist) }; }
#line 1294 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 118:
#line 521 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.aggr) = { (yystack_[2].value.fun), false, BUILDER.bodyaggrelemvec() }; }
#line 1300 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 119:
#line 522 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.aggr) = { (yystack_[3].value.fun), false, (yystack_[1].value.bodyaggrelemvec) }; }
#line 1306 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 120:
#line 526 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.bound) = { Relation::LEQ, (yystack_[0].value.term) }; }
#line 1312 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 121:
#line 527 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.bound) = { (yystack_[1].value.rel), (yystack_[0].value.term) }; }
#line 1318 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 122:
#line 528 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.bound) = { Relation::LEQ, TermUid(-1) }; }
#line 1324 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 123:
#line 532 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.uid) = lexer->aggregate((yystack_[1].value.aggr).fun, (yystack_[1].value.aggr).choice, (yystack_[1].value.aggr).elems, lexer->boundvec(Relation::LEQ, (yystack_[2].value.term), (yystack_[0].value.bound).rel, (yystack_[0].value.bound).term)); }
#line 1330 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 124:
#line 533 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.uid) = lexer->aggregate((yystack_[1].value.aggr).fun, (yystack_[1].value.aggr).choice, (yystack_[1].value.aggr).elems, lexer->boundvec((yystack_[2].value.rel), (yystack_[3].value.term), (yystack_[0].value.bound).rel, (yystack_[0].value.bound).term)); }
#line 1336 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 125:
#line 534 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.uid) = lexer->aggregate((yystack_[1].value.aggr).fun, (yystack_[1].value.aggr).choice, (yystack_[1].value.aggr).elems, lexer->boundvec(Relation::LEQ, TermUid(-1), (yystack_[0].value.bound).rel, (yystack_[0].value.bound).term)); }
#line 1342 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 126:
#line 542 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.headaggrelemvec) = BUILDER.headaggrelemvec((yystack_[5].value.headaggrelemvec), (yystack_[3].value.termvec), (yystack_[1].value.lit), (yystack_[0].value.litvec)); }
#line 1348 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 127:
#line 543 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.headaggrelemvec) = BUILDER.headaggrelemvec(BUILDER.headaggrelemvec(), (yystack_[3].value.termvec), (yystack_[1].value.lit), (yystack_[0].value.litvec)); }
#line 1354 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 128:
#line 547 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.condlitlist) = BUILDER.condlitvec(BUILDER.condlitvec(), (yystack_[1].value.lit), (yystack_[0].value.litvec)); }
#line 1360 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 129:
#line 548 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.condlitlist) = BUILDER.condlitvec((yystack_[3].value.condlitlist), (yystack_[1].value.lit), (yystack_[0].value.litvec)); }
#line 1366 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 130:
#line 554 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.aggr) = { (yystack_[2].value.fun), false, BUILDER.headaggrelemvec() }; }
#line 1372 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 131:
#line 555 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.aggr) = { (yystack_[3].value.fun), false, (yystack_[1].value.headaggrelemvec) }; }
#line 1378 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 132:
#line 556 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.aggr) = { AggregateFunction::COUNT, true, BUILDER.condlitvec()}; }
#line 1384 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 133:
#line 557 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.aggr) = { AggregateFunction::COUNT, true, (yystack_[1].value.condlitlist)}; }
#line 1390 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 134:
#line 561 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.uid) = lexer->aggregate((yystack_[1].value.aggr).fun, (yystack_[1].value.aggr).choice, (yystack_[1].value.aggr).elems, lexer->boundvec(Relation::LEQ, (yystack_[2].value.term), (yystack_[0].value.bound).rel, (yystack_[0].value.bound).term)); }
#line 1396 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 135:
#line 562 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.uid) = lexer->aggregate((yystack_[1].value.aggr).fun, (yystack_[1].value.aggr).choice, (yystack_[1].value.aggr).elems, lexer->boundvec((yystack_[2].value.rel), (yystack_[3].value.term), (yystack_[0].value.bound).rel, (yystack_[0].value.bound).term)); }
#line 1402 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 136:
#line 563 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.uid) = lexer->aggregate((yystack_[1].value.aggr).fun, (yystack_[1].value.aggr).choice, (yystack_[1].value.aggr).elems, lexer->boundvec(Relation::LEQ, TermUid(-1), (yystack_[0].value.bound).rel, (yystack_[0].value.bound).term)); }
#line 1408 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 137:
#line 570 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.cspelemvec) = BUILDER.cspelemvec(BUILDER.cspelemvec(), yylhs.location, (yystack_[3].value.termvec), (yystack_[1].value.cspaddterm), (yystack_[0].value.litvec)); }
#line 1414 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 138:
#line 571 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.cspelemvec) = BUILDER.cspelemvec((yystack_[5].value.cspelemvec), yylhs.location, (yystack_[3].value.termvec), (yystack_[1].value.cspaddterm), (yystack_[0].value.litvec)); }
#line 1420 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 139:
#line 575 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.cspelemvec) = (yystack_[0].value.cspelemvec); }
#line 1426 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 140:
#line 576 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.cspelemvec) = BUILDER.cspelemvec(); }
#line 1432 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 141:
#line 580 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.disjoint) = { NAF::POS, (yystack_[1].value.cspelemvec) }; }
#line 1438 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 142:
#line 581 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.disjoint) = { NAF::NOT, (yystack_[1].value.cspelemvec) }; }
#line 1444 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 143:
#line 582 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.disjoint) = { NAF::NOTNOT, (yystack_[1].value.cspelemvec) }; }
#line 1450 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 144:
#line 589 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.lbodyaggrelem) = { (yystack_[2].value.lit), (yystack_[0].value.litvec) }; }
#line 1456 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 147:
#line 601 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.condlitlist) = BUILDER.condlitvec((yystack_[2].value.condlitlist), (yystack_[1].value.lit), BUILDER.litvec()); }
#line 1462 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 148:
#line 602 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.condlitlist) = BUILDER.condlitvec((yystack_[3].value.condlitlist), (yystack_[2].value.lit), (yystack_[1].value.litvec)); }
#line 1468 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 149:
#line 603 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.condlitlist) = BUILDER.condlitvec(); }
#line 1474 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 150:
#line 608 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.condlitlist) = BUILDER.condlitvec(BUILDER.condlitvec((yystack_[2].value.condlitlist), (yystack_[1].value.lit), (yystack_[0].value.litvec)), (yystack_[4].value.lit), BUILDER.litvec()); }
#line 1480 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 151:
#line 609 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.condlitlist) = BUILDER.condlitvec(BUILDER.condlitvec((yystack_[2].value.condlitlist), (yystack_[1].value.lit), (yystack_[0].value.litvec)), (yystack_[4].value.lit), BUILDER.litvec()); }
#line 1486 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 152:
#line 610 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.condlitlist) = BUILDER.condlitvec(BUILDER.condlitvec((yystack_[2].value.condlitlist), (yystack_[1].value.lit), (yystack_[0].value.litvec)), (yystack_[6].value.lit), (yystack_[4].value.litvec)); }
#line 1492 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 153:
#line 611 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.condlitlist) = BUILDER.condlitvec(BUILDER.condlitvec(), (yystack_[2].value.lit), (yystack_[0].value.litvec)); }
#line 1498 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 154:
#line 620 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = BUILDER.bodylit((yystack_[2].value.body), (yystack_[1].value.lit)); }
#line 1504 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 155:
#line 621 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = BUILDER.bodylit((yystack_[2].value.body), (yystack_[1].value.lit)); }
#line 1510 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 156:
#line 622 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = lexer->bodyaggregate((yystack_[2].value.body), yystack_[1].location, NAF::POS, (yystack_[1].value.uid)); }
#line 1516 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 157:
#line 623 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = lexer->bodyaggregate((yystack_[2].value.body), yystack_[1].location, NAF::POS, (yystack_[1].value.uid)); }
#line 1522 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 158:
#line 624 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = lexer->bodyaggregate((yystack_[3].value.body), yystack_[1].location + yystack_[2].location, NAF::NOT, (yystack_[1].value.uid)); }
#line 1528 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 159:
#line 625 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = lexer->bodyaggregate((yystack_[3].value.body), yystack_[1].location + yystack_[2].location, NAF::NOT, (yystack_[1].value.uid)); }
#line 1534 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 160:
#line 626 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = lexer->bodyaggregate((yystack_[4].value.body), yystack_[1].location + yystack_[3].location, NAF::NOTNOT, (yystack_[1].value.uid)); }
#line 1540 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 161:
#line 627 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = lexer->bodyaggregate((yystack_[4].value.body), yystack_[1].location + yystack_[3].location, NAF::NOTNOT, (yystack_[1].value.uid)); }
#line 1546 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 162:
#line 628 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = BUILDER.conjunction((yystack_[2].value.body), yystack_[1].location, (yystack_[1].value.lbodyaggrelem).first, (yystack_[1].value.lbodyaggrelem).second); }
#line 1552 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 163:
#line 629 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = BUILDER.disjoint((yystack_[2].value.body), yystack_[1].location, (yystack_[1].value.disjoint).first, (yystack_[1].value.disjoint).second); }
#line 1558 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 164:
#line 630 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = BUILDER.body(); }
#line 1564 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 165:
#line 634 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = BUILDER.bodylit((yystack_[2].value.body), (yystack_[1].value.lit)); }
#line 1570 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 166:
#line 635 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = lexer->bodyaggregate((yystack_[2].value.body), yystack_[1].location, NAF::POS, (yystack_[1].value.uid)); }
#line 1576 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 167:
#line 636 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = lexer->bodyaggregate((yystack_[3].value.body), yystack_[1].location + yystack_[2].location, NAF::NOT, (yystack_[1].value.uid)); }
#line 1582 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 168:
#line 637 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = lexer->bodyaggregate((yystack_[4].value.body), yystack_[1].location + yystack_[3].location, NAF::NOTNOT, (yystack_[1].value.uid)); }
#line 1588 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 169:
#line 638 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = BUILDER.conjunction((yystack_[2].value.body), yystack_[1].location, (yystack_[1].value.lbodyaggrelem).first, (yystack_[1].value.lbodyaggrelem).second); }
#line 1594 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 170:
#line 639 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = BUILDER.disjoint((yystack_[2].value.body), yystack_[1].location, (yystack_[1].value.disjoint).first, (yystack_[1].value.disjoint).second); }
#line 1600 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 171:
#line 643 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.head) = BUILDER.headlit((yystack_[0].value.lit)); }
#line 1606 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 172:
#line 644 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.head) = BUILDER.disjunction(yylhs.location, (yystack_[0].value.condlitlist)); }
#line 1612 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 173:
#line 645 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.head) = lexer->headaggregate(yylhs.location, (yystack_[0].value.uid)); }
#line 1618 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 174:
#line 649 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.rule(yylhs.location, (yystack_[1].value.head)); }
#line 1624 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 175:
#line 650 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.rule(yylhs.location, (yystack_[2].value.head), (yystack_[0].value.body)); }
#line 1630 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 176:
#line 651 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.rule(yylhs.location, BUILDER.headlit(BUILDER.boollit(yylhs.location, false)), (yystack_[0].value.body)); }
#line 1636 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 177:
#line 652 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.rule(yylhs.location, BUILDER.headlit(BUILDER.boollit(yylhs.location, false)), BUILDER.body()); }
#line 1642 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 178:
#line 659 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.rule(yylhs.location, BUILDER.headlit(BUILDER.boollit(yystack_[2].location, false)), BUILDER.disjoint((yystack_[0].value.body), yystack_[2].location, inv((yystack_[2].value.disjoint).first), (yystack_[2].value.disjoint).second)); }
#line 1648 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 179:
#line 660 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.rule(yylhs.location, BUILDER.headlit(BUILDER.boollit(yystack_[2].location, false)), BUILDER.disjoint(BUILDER.body(), yystack_[2].location, inv((yystack_[2].value.disjoint).first), (yystack_[2].value.disjoint).second)); }
#line 1654 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 180:
#line 661 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.rule(yylhs.location, BUILDER.headlit(BUILDER.boollit(yystack_[1].location, false)), BUILDER.disjoint(BUILDER.body(), yystack_[1].location, inv((yystack_[1].value.disjoint).first), (yystack_[1].value.disjoint).second)); }
#line 1660 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 181:
#line 668 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.termvec) = (yystack_[0].value.termvec); }
#line 1666 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 182:
#line 669 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.termvec) = BUILDER.termvec(); }
#line 1672 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 183:
#line 673 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.termpair) = {(yystack_[2].value.term), (yystack_[0].value.term)}; }
#line 1678 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 184:
#line 674 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.termpair) = {(yystack_[0].value.term), BUILDER.term(yylhs.location, Value(0))}; }
#line 1684 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 185:
#line 678 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = BUILDER.bodylit(BUILDER.body(), (yystack_[0].value.lit)); }
#line 1690 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 186:
#line 679 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = BUILDER.bodylit((yystack_[2].value.body), (yystack_[0].value.lit)); }
#line 1696 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 187:
#line 683 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = (yystack_[0].value.body); }
#line 1702 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 188:
#line 684 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = BUILDER.body(); }
#line 1708 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 189:
#line 685 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.body) = BUILDER.body(); }
#line 1714 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 190:
#line 689 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.optimize(yylhs.location, (yystack_[2].value.termpair).first, (yystack_[2].value.termpair).second, (yystack_[1].value.termvec), (yystack_[4].value.body)); }
#line 1720 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 191:
#line 690 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.optimize(yylhs.location, (yystack_[2].value.termpair).first, (yystack_[2].value.termpair).second, (yystack_[1].value.termvec), BUILDER.body()); }
#line 1726 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 192:
#line 694 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.optimize(yylhs.location, BUILDER.term(yystack_[2].location, UnOp::NEG, (yystack_[2].value.termpair).first), (yystack_[2].value.termpair).second, (yystack_[1].value.termvec), (yystack_[0].value.body)); }
#line 1732 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 193:
#line 695 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.optimize(yylhs.location, BUILDER.term(yystack_[2].location, UnOp::NEG, (yystack_[2].value.termpair).first), (yystack_[2].value.termpair).second, (yystack_[1].value.termvec), (yystack_[0].value.body)); }
#line 1738 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 194:
#line 699 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.optimize(yylhs.location, (yystack_[2].value.termpair).first, (yystack_[2].value.termpair).second, (yystack_[1].value.termvec), (yystack_[0].value.body)); }
#line 1744 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 195:
#line 700 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.optimize(yylhs.location, (yystack_[2].value.termpair).first, (yystack_[2].value.termpair).second, (yystack_[1].value.termvec), (yystack_[0].value.body)); }
#line 1750 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 200:
#line 714 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.showsig(yylhs.location, (yystack_[3].value.uid), (yystack_[1].value.num), false); }
#line 1756 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 201:
#line 715 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.showsig(yylhs.location, "-"+*FWString((yystack_[3].value.uid)), (yystack_[1].value.num), false); }
#line 1762 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 202:
#line 716 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.showsig(yylhs.location, "", 0, false); }
#line 1768 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 203:
#line 717 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.show(yylhs.location, (yystack_[2].value.term), (yystack_[0].value.body), false); }
#line 1774 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 204:
#line 718 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.show(yylhs.location, (yystack_[1].value.term), BUILDER.body(), false); }
#line 1780 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 205:
#line 719 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.showsig(yylhs.location, (yystack_[3].value.uid), (yystack_[1].value.num), true); }
#line 1786 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 206:
#line 720 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.show(yylhs.location, (yystack_[2].value.term), (yystack_[0].value.body), true); }
#line 1792 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 207:
#line 721 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.show(yylhs.location, (yystack_[1].value.term), BUILDER.body(), true); }
#line 1798 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 208:
#line 728 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    {  BUILDER.define(yylhs.location, (yystack_[2].value.uid), (yystack_[0].value.term), false); }
#line 1804 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 209:
#line 732 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    {  BUILDER.define(yylhs.location, (yystack_[3].value.uid), (yystack_[1].value.term), true); }
#line 1810 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 210:
#line 739 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.python(yylhs.location, (yystack_[1].value.uid)); }
#line 1816 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 211:
#line 740 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.lua(yylhs.location, (yystack_[1].value.uid)); }
#line 1822 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 212:
#line 747 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { lexer->include((yystack_[1].value.uid), yylhs.location, false); }
#line 1828 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 213:
#line 748 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { lexer->include((yystack_[2].value.uid), yylhs.location, true); }
#line 1834 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 214:
#line 755 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.idlist) = BUILDER.idvec((yystack_[2].value.idlist), yystack_[0].location, (yystack_[0].value.uid)); }
#line 1840 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 215:
#line 756 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.idlist) = BUILDER.idvec(BUILDER.idvec(), yystack_[0].location, (yystack_[0].value.uid)); }
#line 1846 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 216:
#line 760 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.idlist) = BUILDER.idvec(); }
#line 1852 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 217:
#line 761 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { (yylhs.value.idlist) = (yystack_[0].value.idlist); }
#line 1858 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 218:
#line 765 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.block(yylhs.location, (yystack_[4].value.uid), (yystack_[2].value.idlist)); }
#line 1864 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 219:
#line 766 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.block(yylhs.location, (yystack_[1].value.uid), BUILDER.idvec()); }
#line 1870 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 220:
#line 773 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.external(yylhs.location, BUILDER.predlit(yystack_[2].location, NAF::POS, (yystack_[2].value.pair).second & 1, FWString((yystack_[2].value.pair).first), TermVecVecUid((yystack_[2].value.pair).second >> 1u)), (yystack_[0].value.body)); }
#line 1876 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 221:
#line 774 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.external(yylhs.location, BUILDER.predlit(yystack_[2].location, NAF::POS, (yystack_[2].value.pair).second & 1, FWString((yystack_[2].value.pair).first), TermVecVecUid((yystack_[2].value.pair).second >> 1u)), BUILDER.body()); }
#line 1882 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;

  case 222:
#line 775 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:847
    { BUILDER.external(yylhs.location, BUILDER.predlit(yystack_[1].location, NAF::POS, (yystack_[1].value.pair).second & 1, FWString((yystack_[1].value.pair).first), TermVecVecUid((yystack_[1].value.pair).second >> 1u)), BUILDER.body()); }
#line 1888 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
    break;


#line 1892 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:847
            default:
              break;
            }
        }
      catch (const syntax_error& yyexc)
        {
          error (yyexc);
          YYERROR;
        }
      YY_SYMBOL_PRINT ("-> $$ =", yylhs);
      yypop_ (yylen);
      yylen = 0;
      YY_STACK_PRINT ();

      // Shift the result of the reduction.
      yypush_ (YY_NULLPTR, yylhs);
    }
    goto yynewstate;

  /*--------------------------------------.
  | yyerrlab -- here on detecting error.  |
  `--------------------------------------*/
  yyerrlab:
    // If not already recovering from an error, report this error.
    if (!yyerrstatus_)
      {
        ++yynerrs_;
        error (yyla.location, yysyntax_error_ (yystack_[0].state,
                                           yyempty ? yyempty_ : yyla.type_get ()));
      }


    yyerror_range[1].location = yyla.location;
    if (yyerrstatus_ == 3)
      {
        /* If just tried and failed to reuse lookahead token after an
           error, discard it.  */

        // Return failure if at end of input.
        if (yyla.type_get () == yyeof_)
          YYABORT;
        else if (!yyempty)
          {
            yy_destroy_ ("Error: discarding", yyla);
            yyempty = true;
          }
      }

    // Else will try to reuse lookahead token after shifting the error token.
    goto yyerrlab1;


  /*---------------------------------------------------.
  | yyerrorlab -- error raised explicitly by YYERROR.  |
  `---------------------------------------------------*/
  yyerrorlab:

    /* Pacify compilers like GCC when the user code never invokes
       YYERROR and the label yyerrorlab therefore never appears in user
       code.  */
    if (false)
      goto yyerrorlab;
    yyerror_range[1].location = yystack_[yylen - 1].location;
    /* Do not reclaim the symbols of the rule whose action triggered
       this YYERROR.  */
    yypop_ (yylen);
    yylen = 0;
    goto yyerrlab1;

  /*-------------------------------------------------------------.
  | yyerrlab1 -- common code for both syntax error and YYERROR.  |
  `-------------------------------------------------------------*/
  yyerrlab1:
    yyerrstatus_ = 3;   // Each real token shifted decrements this.
    {
      stack_symbol_type error_token;
      for (;;)
        {
          yyn = yypact_[yystack_[0].state];
          if (!yy_pact_value_is_default_ (yyn))
            {
              yyn += yyterror_;
              if (0 <= yyn && yyn <= yylast_ && yycheck_[yyn] == yyterror_)
                {
                  yyn = yytable_[yyn];
                  if (0 < yyn)
                    break;
                }
            }

          // Pop the current state because it cannot handle the error token.
          if (yystack_.size () == 1)
            YYABORT;

          yyerror_range[1].location = yystack_[0].location;
          yy_destroy_ ("Error: popping", yystack_[0]);
          yypop_ ();
          YY_STACK_PRINT ();
        }

      yyerror_range[2].location = yyla.location;
      YYLLOC_DEFAULT (error_token.location, yyerror_range, 2);

      // Shift the error token.
      error_token.state = yyn;
      yypush_ ("Shifting", error_token);
    }
    goto yynewstate;

    // Accept.
  yyacceptlab:
    yyresult = 0;
    goto yyreturn;

    // Abort.
  yyabortlab:
    yyresult = 1;
    goto yyreturn;

  yyreturn:
    if (!yyempty)
      yy_destroy_ ("Cleanup: discarding lookahead", yyla);

    /* Do not reclaim the symbols of the rule whose action triggered
       this YYABORT or YYACCEPT.  */
    yypop_ (yylen);
    while (1 < yystack_.size ())
      {
        yy_destroy_ ("Cleanup: popping", yystack_[0]);
        yypop_ ();
      }

    return yyresult;
  }
    catch (...)
      {
        YYCDEBUG << "Exception caught: cleaning lookahead and stack"
                 << std::endl;
        // Do not try to display the values of the reclaimed symbols,
        // as their printer might throw an exception.
        if (!yyempty)
          yy_destroy_ (YY_NULLPTR, yyla);

        while (1 < yystack_.size ())
          {
            yy_destroy_ (YY_NULLPTR, yystack_[0]);
            yypop_ ();
          }
        throw;
      }
  }

  void
  parser::error (const syntax_error& yyexc)
  {
    error (yyexc.location, yyexc.what());
  }

  // Generate an error message.
  std::string
  parser::yysyntax_error_ (state_type yystate, symbol_number_type yytoken) const
  {
    std::string yyres;
    // Number of reported tokens (one for the "unexpected", one per
    // "expected").
    size_t yycount = 0;
    // Its maximum.
    enum { YYERROR_VERBOSE_ARGS_MAXIMUM = 5 };
    // Arguments of yyformat.
    char const *yyarg[YYERROR_VERBOSE_ARGS_MAXIMUM];

    /* There are many possibilities here to consider:
       - If this state is a consistent state with a default action, then
         the only way this function was invoked is if the default action
         is an error action.  In that case, don't check for expected
         tokens because there are none.
       - The only way there can be no lookahead present (in yytoken) is
         if this state is a consistent state with a default action.
         Thus, detecting the absence of a lookahead is sufficient to
         determine that there is no unexpected or expected token to
         report.  In that case, just report a simple "syntax error".
       - Don't assume there isn't a lookahead just because this state is
         a consistent state with a default action.  There might have
         been a previous inconsistent state, consistent state with a
         non-default action, or user semantic action that manipulated
         yyla.  (However, yyla is currently not documented for users.)
       - Of course, the expected token list depends on states to have
         correct lookahead information, and it depends on the parser not
         to perform extra reductions after fetching a lookahead from the
         scanner and before detecting a syntax error.  Thus, state
         merging (from LALR or IELR) and default reductions corrupt the
         expected token list.  However, the list is correct for
         canonical LR with one exception: it will still contain any
         token that will not be accepted due to an error action in a
         later state.
    */
    if (yytoken != yyempty_)
      {
        yyarg[yycount++] = yytname_[yytoken];
        int yyn = yypact_[yystate];
        if (!yy_pact_value_is_default_ (yyn))
          {
            /* Start YYX at -YYN if negative to avoid negative indexes in
               YYCHECK.  In other words, skip the first -YYN actions for
               this state because they are default actions.  */
            int yyxbegin = yyn < 0 ? -yyn : 0;
            // Stay within bounds of both yycheck and yytname.
            int yychecklim = yylast_ - yyn + 1;
            int yyxend = yychecklim < yyntokens_ ? yychecklim : yyntokens_;
            for (int yyx = yyxbegin; yyx < yyxend; ++yyx)
              if (yycheck_[yyx + yyn] == yyx && yyx != yyterror_
                  && !yy_table_value_is_error_ (yytable_[yyx + yyn]))
                {
                  if (yycount == YYERROR_VERBOSE_ARGS_MAXIMUM)
                    {
                      yycount = 1;
                      break;
                    }
                  else
                    yyarg[yycount++] = yytname_[yyx];
                }
          }
      }

    char const* yyformat = YY_NULLPTR;
    switch (yycount)
      {
#define YYCASE_(N, S)                         \
        case N:                               \
          yyformat = S;                       \
        break
        YYCASE_(0, YY_("syntax error"));
        YYCASE_(1, YY_("syntax error, unexpected %s"));
        YYCASE_(2, YY_("syntax error, unexpected %s, expecting %s"));
        YYCASE_(3, YY_("syntax error, unexpected %s, expecting %s or %s"));
        YYCASE_(4, YY_("syntax error, unexpected %s, expecting %s or %s or %s"));
        YYCASE_(5, YY_("syntax error, unexpected %s, expecting %s or %s or %s or %s"));
#undef YYCASE_
      }

    // Argument number.
    size_t yyi = 0;
    for (char const* yyp = yyformat; *yyp; ++yyp)
      if (yyp[0] == '%' && yyp[1] == 's' && yyi < yycount)
        {
          yyres += yytnamerr_ (yyarg[yyi++]);
          ++yyp;
        }
      else
        yyres += *yyp;
    return yyres;
  }


  const short int parser::yypact_ninf_ = -314;

  const signed char parser::yytable_ninf_ = -74;

  const short int
  parser::yypact_[] =
  {
     251,  -314,   -20,    55,   596,  -314,    33,  -314,  -314,    32,
     -20,  1154,   -20,  -314,  1154,    23,   -12,  -314,    53,    -1,
    -314,   737,  1154,  -314,    44,  -314,    66,   940,    41,  1154,
    -314,  -314,  -314,  -314,    54,  1154,    92,  -314,  -314,    96,
     104,  -314,  -314,   116,  -314,   738,  1197,  -314,    40,  -314,
     952,   397,    95,   702,  -314,    27,  -314,    93,   440,  -314,
      97,  1154,   103,  -314,   153,   478,   984,   -20,   123,    42,
    -314,   460,  -314,   101,   141,  -314,    99,   525,   174,    58,
    1342,   185,  -314,   232,  1023,  1032,  1154,  -314,   315,   135,
     164,   183,  1344,  -314,    38,  1342,    11,   210,   231,  -314,
    -314,   242,    13,  -314,  1154,  1154,  1154,  -314,   281,  1154,
    -314,  -314,  -314,  -314,  -314,  1154,  1154,  -314,  1154,  1154,
    1154,  1154,  1154,   859,   702,   230,  -314,  -314,  -314,  -314,
    1071,  1071,  -314,  -314,  -314,  -314,  -314,  -314,  1071,  1071,
    1106,  1342,  1154,  -314,  -314,   275,  -314,  -314,   -20,   440,
    -314,   440,   440,  -314,   440,  -314,  -314,   266,   565,  1154,
    1154,   440,  1154,   301,  -314,   107,   288,  1154,   304,  -314,
     777,   654,  1245,   217,   295,   702,    37,    15,    19,   306,
    -314,   -12,  1154,   230,  -314,  -314,   230,  1154,  -314,  1154,
     330,   215,   347,   143,   334,   347,   156,  1318,  -314,  -314,
     319,   321,   291,  1154,  -314,   300,  1154,  -314,  1154,  1154,
     984,   344,  -314,   293,   168,   119,  1154,  1149,   348,   348,
     348,   381,   348,   168,   333,  1342,   702,  -314,  -314,    26,
     230,   230,   712,  -314,  -314,   335,   335,  -314,   383,   167,
    1342,  -314,  -314,  -314,   360,  -314,   565,   401,   371,  -314,
      39,   440,   440,   440,   440,   440,   440,   440,   440,   440,
     440,   299,   317,   406,  1342,  1071,  -314,  1154,  1154,   355,
    -314,  -314,  -314,   174,  -314,   182,   787,  1293,   263,   868,
     702,   230,  -314,  -314,  -314,   949,  -314,  -314,  -314,  -314,
    -314,  -314,  -314,  -314,   400,   418,  -314,   174,  1342,  -314,
    -314,  1154,  1154,   424,   410,  1154,  -314,   424,   420,  1154,
    -314,  -314,  -314,   372,   376,   425,   387,  -314,   446,   413,
    1342,   347,   347,   195,   984,   289,  1342,  -314,   230,  -314,
     449,   449,   230,  -314,  1154,   440,   440,  -314,  -314,   415,
     267,   177,   422,   422,   422,   991,   422,   267,   836,  -314,
    -314,  -314,   173,   469,   408,  -314,  -314,  -314,   230,   272,
     606,  -314,  -314,  -314,   702,  -314,  -314,   230,  -314,   467,
    -314,   239,  -314,  -314,  1342,   185,   230,  -314,  -314,   347,
    -314,  -314,   347,  -314,   455,   458,  -314,   671,   429,   468,
     456,   459,  -314,   294,  -314,   230,   230,  -314,    48,    48,
     174,   497,   457,   565,  -314,  -314,  1071,  -314,  -314,  -314,
    -314,  -314,  -314,  -314,  -314,  -314,  1115,  -314,   502,   424,
     424,  -314,  -314,  -314,  -314,  -314,  -314,  -314,   449,   418,
    -314,  -314,   230,  -314,   173,  -314,   230,  -314,  -314,    48,
     174,  -314,  -314,  -314
  };

  const unsigned char
  parser::yydefact_[] =
  {
       0,     5,     0,     0,     0,     7,     0,     3,     1,     0,
       0,     0,     0,   108,     0,     0,     0,    75,   164,     0,
      51,     0,    60,   107,     0,   106,     0,     0,     0,     0,
     104,   105,    52,    74,     0,     0,   164,    49,    54,     0,
       0,    50,    53,     0,     4,    48,    84,    76,   171,    87,
       0,    80,     0,   122,   173,     0,   172,     0,     0,     6,
       0,     0,    48,    43,     0,    83,   140,     0,    70,     0,
     177,     0,   176,     0,     0,   132,     0,    84,   101,     0,
      57,    59,    61,     0,     0,     0,     0,   202,     0,     0,
       0,     0,    48,    42,     0,    55,     0,     0,     0,   210,
     211,     0,     0,    77,    60,     0,     0,    69,     0,     0,
      67,    65,    63,    66,    64,     0,     0,    68,     0,     0,
       0,     0,     0,     0,   122,     0,   149,   145,   146,   149,
       0,     0,    91,    89,    88,    90,    92,    93,     0,     0,
      60,   120,     0,   136,   180,   164,   174,   164,     0,     0,
      26,    31,     0,    27,     0,    24,    25,    23,   208,    60,
      60,     0,     0,     0,   139,     0,    72,    60,   164,   222,
       0,     0,    84,     0,     0,   122,     0,     0,     0,     0,
     212,     0,     0,    99,   128,   133,     0,     0,    44,    60,
       0,   184,   182,     0,     0,   182,     0,     0,   164,   204,
       0,     0,     0,    60,   219,   216,     0,    47,     0,     0,
     140,     0,    78,     0,    36,    35,     0,    32,    40,    38,
      41,    34,    39,    37,    33,    79,   122,   134,    96,   153,
       0,     0,    84,    85,    86,    95,    94,   130,     0,     0,
     121,   179,   178,   175,     0,    18,    28,    30,     0,    17,
       0,    31,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,     0,    81,     0,   141,    60,    60,     0,
     221,   220,   116,   101,   114,     0,     0,     0,     0,     0,
     122,    99,   154,   165,   155,     0,   125,   156,   166,   157,
     170,   163,   169,   162,     0,    98,   100,   101,    58,    62,
     197,     0,     0,   189,     0,     0,   196,   189,     0,     0,
     164,   207,   203,     0,     0,     0,     0,   215,   217,     0,
      56,   182,   182,     0,   140,    45,    82,   135,     0,   149,
     103,   103,     0,   131,    60,    31,     0,    19,    22,     0,
      11,    10,    15,    13,    16,     9,    14,    12,     8,    46,
      45,   209,   101,     0,     0,    71,   113,   117,     0,     0,
       0,   158,   167,   159,   122,   123,   144,    99,   118,   101,
     111,     0,   213,   129,   183,   181,   188,   192,   199,   182,
     194,   198,   182,   206,     0,     0,   200,    45,     0,     0,
       0,     0,   142,     0,    97,     0,     0,   147,   150,   151,
     101,     0,     0,    29,    20,   137,     0,    73,   115,   160,
     168,   161,   124,   109,   110,   119,     0,   185,   187,   189,
     189,   205,   201,   214,   218,   191,   190,   143,   103,   102,
     148,   127,     0,    21,   101,   112,     0,   193,   195,   152,
     101,   138,   186,   126
  };

  const short int
  parser::yypgoto_[] =
  {
    -314,  -314,  -314,  -314,    -2,   -55,  -314,  -216,   282,  -314,
    -268,   -57,   -78,   -31,   -10,     0,   356,  -126,   463,  -314,
    -120,  -251,  -249,  -313,    12,   106,  -314,   157,  -314,  -149,
    -102,  -142,  -314,  -314,   -14,  -314,  -314,  -179,   471,  -314,
     -37,  -122,  -314,  -314,   -35,  -314,  -167,   -65,  -314,  -282,
    -314,  -314,  -314,  -314,  -314
  };

  const short int
  parser::yydefgoto_[] =
  {
      -1,     3,     4,    44,    62,   246,   247,   248,    77,    96,
      81,    82,    83,   142,    47,   228,    49,    50,   138,    51,
     295,   296,   184,   398,   174,   370,   371,   274,   275,   175,
     143,   176,   239,    79,    53,    54,   164,   165,    55,   178,
     430,   230,    56,    71,    72,    57,   303,   192,   418,   377,
     193,   196,     7,   318,   319
  };

  const short int
  parser::yytable_[] =
  {
       6,    98,    45,   158,    48,   229,    69,   231,    60,   163,
      64,   129,   235,   236,    68,   123,    52,   369,   399,    45,
     195,    78,   227,   280,   356,   380,   213,    92,   307,   278,
     366,   323,   124,   103,   375,   339,   328,   211,    58,    73,
     290,    68,   252,   253,   292,    67,   182,   287,   373,   125,
     126,   168,   144,     5,    89,     8,   157,    59,    52,    66,
     145,     5,   288,   204,   206,   166,   103,   169,   291,    45,
      67,   173,   293,   286,    68,    74,   207,   205,    70,   127,
      84,   261,   262,   238,   254,   255,     5,   256,   257,   269,
     289,   128,   212,   127,   245,   258,   259,   249,    90,   250,
      68,   127,    85,   405,   338,   128,   263,   260,   185,   226,
     242,   186,   243,   128,    91,   439,   413,    97,   146,   402,
     414,    99,   105,    45,   327,   316,   147,    94,   280,   100,
     364,   140,   299,   271,   359,    52,   159,   437,   438,   352,
     101,   279,   160,   321,   322,   393,   244,   157,   369,   157,
     157,   431,   157,   163,   390,   391,    67,   266,   161,   157,
     267,   103,   167,   312,   115,   116,   180,   118,    45,    45,
     273,   212,     5,    67,   179,   120,   121,   181,   365,    68,
     252,    45,   183,   183,    45,   441,   297,   130,   131,     5,
     354,   443,   329,   304,   102,   187,   305,   340,   341,   342,
     343,   344,   345,   346,   347,   348,   308,   395,   200,   309,
     353,   364,   419,   115,   116,   420,   118,   333,   105,   106,
     334,   301,   254,   255,   120,   256,   281,   282,    45,    45,
     330,   331,   357,   258,   259,   358,    10,   201,    11,   202,
     379,   109,   283,    14,   382,   392,   360,   208,   267,   157,
     157,   157,   157,   157,   157,   157,   157,   157,   157,    17,
     115,   116,   412,   118,   119,    20,   212,   163,   209,    22,
     284,   120,   121,   361,    45,   383,   429,   401,   210,    45,
     434,   403,   409,   122,   188,   189,    46,    29,   362,   415,
      32,    33,   416,    63,   216,    35,    65,   410,   -71,   -71,
     241,    37,    38,     5,    80,   251,    41,    42,    76,    88,
     265,    93,   254,   255,   -71,   256,   363,    95,   105,   106,
       1,     2,   -71,   258,   198,   411,    45,   268,   394,   270,
      45,   285,   400,   157,   157,   141,   105,   106,   294,   -71,
     199,   109,   -71,    93,   427,   325,   189,   267,    80,   130,
     131,   349,   189,   172,   -71,   300,    45,   302,   273,   306,
     115,   116,   315,   118,   119,    45,   191,   191,   197,   350,
     189,   120,   121,   317,    45,   313,   417,   314,   115,   116,
     324,   118,   119,   122,   105,   106,    80,   214,   215,   120,
     121,   217,   332,    45,    45,   428,   118,   218,   219,   335,
     220,   221,   222,   223,   224,   225,   141,   355,   189,   252,
     253,   336,   232,   232,   132,   133,   134,   135,   136,   137,
     232,   232,    80,   337,   240,   372,   115,   116,   328,   118,
      45,   351,   440,   376,    45,   378,   442,   120,   121,   387,
     189,    80,    80,   384,   264,   381,   148,   385,   149,    80,
     386,   254,   255,   277,   256,   257,   388,   141,   396,   397,
     407,   189,   258,   259,   225,   389,    10,   404,    11,   298,
     256,    80,    13,    14,   260,   150,   183,   187,   406,   151,
     421,   105,   106,   422,    15,    80,   233,   234,   320,    17,
     191,   191,    80,   424,   162,    20,   170,   152,   326,    22,
     153,    23,   423,    25,   109,   154,   432,   425,   141,   433,
     426,   155,   436,     5,   139,   408,   156,    29,    30,    31,
      32,    33,   435,   115,   116,    35,   118,   119,   105,   106,
     107,    37,    38,     5,   120,   121,    41,    42,   171,     0,
       0,   108,   177,     0,     0,     0,   122,   232,     0,    80,
      80,   109,   110,     0,     0,     0,   111,   112,   277,     0,
       0,   225,   141,   113,     0,   114,     0,    80,   252,   253,
     115,   116,   117,   118,   119,     0,     0,     0,     0,     0,
       0,   120,   121,   374,    80,     0,     0,   191,     0,     0,
       0,   191,     0,   122,     0,     0,    -2,     9,     0,     0,
       0,     0,    10,     0,    11,     0,    80,    12,    13,    14,
     254,   255,     0,   256,   257,     0,    80,     0,    13,     0,
      15,   258,   259,     0,    16,    17,     0,     0,     0,    18,
      19,    20,    21,   260,     0,    22,     0,    23,    24,    25,
      26,     0,   170,     0,     0,     0,   141,    23,     0,    25,
      27,    28,     0,    29,    30,    31,    32,    33,    34,     0,
      10,    35,    11,    36,    30,    31,    13,    37,    38,     5,
      39,    40,    41,    42,    43,     0,     0,     0,   101,     0,
     -73,   -73,     0,     0,     0,     0,     0,     0,   232,    20,
     170,     0,     0,    22,     0,    23,   -73,    25,    80,     0,
       0,     0,     0,     0,   -73,     0,     0,   107,    10,     0,
      11,    29,    30,    31,    32,   105,   106,     0,     0,    35,
       0,   -73,     0,     0,   -73,    37,    38,     5,   108,   110,
      41,    42,   276,   111,   112,     0,   -73,    20,   109,     0,
     113,    22,   114,    10,     0,    11,     0,   -70,   -70,   117,
      14,     0,     0,     0,     0,     0,     0,   115,   116,    61,
     118,   119,    32,   -70,     0,     0,    17,    35,   120,   121,
       0,   -70,    20,    37,    38,     5,    22,   104,    41,    42,
     122,     0,     0,    10,     0,    11,     0,    75,   -70,     0,
      14,   -70,     0,    10,    29,    11,     0,    32,    33,    13,
       0,     0,    35,   -70,     0,     0,    17,     0,    37,    38,
       5,   211,    20,    41,    42,    76,    22,     0,     0,     0,
       0,     0,    20,   170,     0,     0,    22,   272,    23,     0,
      25,     0,     0,     0,    29,     0,     0,    32,    33,   252,
     253,     0,    35,     0,    29,    30,    31,    32,    37,    38,
       5,     0,    35,    41,    42,    76,     0,     0,    37,    38,
       5,     0,     0,    41,    42,    10,     0,    11,     0,     0,
       0,    13,     0,     0,    10,     0,    11,     0,     0,     0,
      13,   254,   255,     0,   256,   257,     0,     0,     0,     0,
       0,     0,   258,   259,    20,    21,     0,     0,    22,     0,
      23,     0,    25,    20,   170,     0,     0,    22,     0,    23,
       0,    25,     0,     0,     0,     0,    61,    30,    31,    32,
       0,     0,     0,     0,    35,    61,    30,    31,    32,     0,
      37,    38,     5,    35,     0,    41,    42,     0,     0,    37,
      38,     5,     0,     0,    41,    42,    10,     0,    11,     0,
       0,     0,     0,    86,     0,    10,     0,    11,   367,     0,
       0,     0,     0,     0,     0,    87,   130,   131,     0,   132,
     133,   134,   135,   136,   137,    20,     0,     0,     0,    22,
       0,     0,     0,     0,    20,     0,     0,     0,    22,     0,
      10,     0,    11,   -60,   252,   253,     0,    61,     0,   368,
      32,     0,     0,     0,     0,    35,    61,     0,     0,    32,
       0,    37,    38,     5,    35,     0,    41,    42,     0,    20,
      37,    38,     5,    22,     0,    41,    42,     0,     0,    10,
       0,    11,     0,     0,     0,     0,   254,   255,    10,   256,
      11,    61,     0,     0,    32,     0,     0,   258,   259,    35,
       0,     0,     0,     0,     0,    37,    38,     5,    20,     0,
      41,    42,    22,     0,     0,     0,     0,    20,     0,     0,
       0,    22,     0,   190,     0,     0,     0,    10,     0,    11,
      61,     0,   194,    32,    14,     0,     0,     0,    35,    61,
       0,     0,    32,     0,    37,    38,     5,    35,     0,    41,
      42,     0,     0,    37,    38,     5,    20,     0,    41,    42,
      22,     0,    10,     0,    11,     0,     0,     0,     0,     0,
       0,    10,     0,    11,   367,     0,     0,     0,    61,     0,
       0,    32,     0,     0,     0,     0,    35,     0,     0,     0,
       0,    20,    37,    38,     5,    22,     0,    41,    42,     0,
      20,     0,   105,   106,    22,     0,   237,     0,     0,     0,
      10,     0,    11,    61,     0,     0,    32,     0,     0,     0,
       0,    35,    61,     0,     0,    32,     0,    37,    38,     5,
      35,     0,    41,    42,     0,     0,    37,    38,     5,    20,
       0,    41,    42,    22,   115,   116,     0,   118,   119,     0,
     105,   106,   107,     0,     0,   120,   121,     0,     0,    13,
       0,    61,     0,   108,    32,     0,     0,   122,     0,    35,
       0,     0,     0,   109,   110,    37,    38,     5,   111,   112,
      41,    42,     0,    21,     0,   113,     0,   114,    23,     0,
      25,     0,   115,   116,   117,   118,   119,     0,   105,   106,
     107,     0,     0,   120,   121,    30,    31,    13,     0,     0,
       0,   108,     0,     0,     0,   122,     0,     0,     0,     0,
       0,   109,   110,     0,     0,     0,   111,   112,     0,     0,
       0,   170,     0,   113,     0,   114,    23,     0,    25,     0,
     115,   116,   117,   118,   119,     0,   105,   106,   107,     0,
       0,   120,   121,    30,    31,    13,     0,     0,     0,     0,
       0,     0,     0,   122,     0,     0,     0,     0,     0,   109,
     110,   105,   106,     0,   111,   112,     0,   310,     0,   170,
       0,   113,     0,   114,    23,     0,    25,     0,   115,   116,
     117,   118,   119,   311,   109,   105,   106,     0,     0,   120,
     121,    30,    31,   -72,   -72,     0,     0,     0,     0,     0,
       0,   122,     0,   115,   116,     0,   118,   119,   109,   -72,
       0,     0,     0,     0,   120,   121,     0,   -72,     0,     0,
       0,     0,     0,   203,     0,     0,   122,   115,   116,     0,
     118,   119,     0,     0,   -72,     0,     0,   -72,   120,   121,
       0,     0,     0,     0,     0,     0,     0,     0,     0,   -72,
     122
  };

  const short int
  parser::yycheck_[] =
  {
       2,    36,     4,    58,     4,   125,    16,   129,    10,    66,
      12,    48,   138,   139,    16,    46,     4,   285,   331,    21,
      85,    21,   124,   172,   273,   307,   104,    29,   195,   171,
     281,   210,    46,    43,   302,   251,    10,    24,     5,    40,
      25,    43,     3,     4,    25,    57,    77,    10,   297,     9,
      10,     9,    25,    73,    13,     0,    58,    25,    46,    36,
      33,    73,    25,    25,    53,    67,    76,    25,    53,    71,
      57,    71,    53,   175,    76,    76,    65,    39,    25,    53,
      36,   159,   160,   140,    45,    46,    73,    48,    49,   167,
      53,    65,   102,    53,   149,    56,    57,   152,    57,   154,
     102,    53,    36,   352,    65,    65,   161,    68,    50,   123,
     145,    53,   147,    65,    73,   428,   367,    25,    25,   335,
     369,    25,     3,   125,   226,   203,    33,    73,   277,    25,
     279,    36,   189,   168,   276,   123,    39,   419,   420,   265,
      24,   172,    39,   208,   209,   324,   148,   149,   416,   151,
     152,   400,   154,   210,   321,   322,    57,    50,     5,   161,
      53,   171,    39,   198,    45,    46,    25,    48,   170,   171,
     170,   181,    73,    57,    73,    56,    57,    78,   280,   181,
       3,   183,     9,     9,   186,   434,   186,    14,    15,    73,
     268,   440,   229,    50,    78,    10,    53,   252,   253,   254,
     255,   256,   257,   258,   259,   260,    50,   329,    73,    53,
     267,   360,   379,    45,    46,   382,    48,    50,     3,     4,
      53,     6,    45,    46,    56,    48,     9,    10,   230,   231,
     230,   231,    50,    56,    57,    53,     6,    73,     8,    56,
     305,    26,    25,    13,   309,    50,   277,    37,    53,   251,
     252,   253,   254,   255,   256,   257,   258,   259,   260,    29,
      45,    46,   364,    48,    49,    35,   276,   324,    37,    39,
      53,    56,    57,    10,   276,   310,   396,   334,    36,   281,
     406,   336,    10,    68,    52,    53,     4,    57,    25,    50,
      60,    61,    53,    11,    13,    65,    14,    25,     9,    10,
      25,    71,    72,    73,    22,    39,    76,    77,    78,    27,
       9,    29,    45,    46,    25,    48,    53,    35,     3,     4,
      69,    70,    33,    56,     9,    53,   328,    39,   328,    25,
     332,    36,   332,   335,   336,    53,     3,     4,    32,    50,
      25,    26,    53,    61,    50,    52,    53,    53,    66,    14,
      15,    52,    53,    71,    65,    25,   358,    10,   358,    25,
      45,    46,    71,    48,    49,   367,    84,    85,    86,    52,
      53,    56,    57,    73,   376,    56,   376,    56,    45,    46,
      36,    48,    49,    68,     3,     4,   104,   105,   106,    56,
      57,   109,     9,   395,   396,   395,    48,   115,   116,    39,
     118,   119,   120,   121,   122,   123,   124,    52,    53,     3,
       4,    10,   130,   131,    17,    18,    19,    20,    21,    22,
     138,   139,   140,    52,   142,    25,    45,    46,    10,    48,
     432,    25,   432,     9,   436,    25,   436,    56,    57,    52,
      53,   159,   160,    71,   162,    25,     6,    71,     8,   167,
      25,    45,    46,   171,    48,    49,    10,   175,     9,    10,
      52,    53,    56,    57,   182,    52,     6,    52,     8,   187,
      48,   189,    12,    13,    68,    35,     9,    10,     9,    39,
      25,     3,     4,    25,    24,   203,   130,   131,   206,    29,
     208,   209,   210,    25,    16,    35,    36,    57,   216,    39,
      60,    41,    73,    43,    26,    65,     9,    51,   226,    52,
      51,    71,    10,    73,    51,   358,    76,    57,    58,    59,
      60,    61,   416,    45,    46,    65,    48,    49,     3,     4,
       5,    71,    72,    73,    56,    57,    76,    77,    78,    -1,
      -1,    16,    71,    -1,    -1,    -1,    68,   265,    -1,   267,
     268,    26,    27,    -1,    -1,    -1,    31,    32,   276,    -1,
      -1,   279,   280,    38,    -1,    40,    -1,   285,     3,     4,
      45,    46,    47,    48,    49,    -1,    -1,    -1,    -1,    -1,
      -1,    56,    57,   301,   302,    -1,    -1,   305,    -1,    -1,
      -1,   309,    -1,    68,    -1,    -1,     0,     1,    -1,    -1,
      -1,    -1,     6,    -1,     8,    -1,   324,    11,    12,    13,
      45,    46,    -1,    48,    49,    -1,   334,    -1,    12,    -1,
      24,    56,    57,    -1,    28,    29,    -1,    -1,    -1,    33,
      34,    35,    36,    68,    -1,    39,    -1,    41,    42,    43,
      44,    -1,    36,    -1,    -1,    -1,   364,    41,    -1,    43,
      54,    55,    -1,    57,    58,    59,    60,    61,    62,    -1,
       6,    65,     8,    67,    58,    59,    12,    71,    72,    73,
      74,    75,    76,    77,    78,    -1,    -1,    -1,    24,    -1,
       9,    10,    -1,    -1,    -1,    -1,    -1,    -1,   406,    35,
      36,    -1,    -1,    39,    -1,    41,    25,    43,   416,    -1,
      -1,    -1,    -1,    -1,    33,    -1,    -1,     5,     6,    -1,
       8,    57,    58,    59,    60,     3,     4,    -1,    -1,    65,
      -1,    50,    -1,    -1,    53,    71,    72,    73,    16,    27,
      76,    77,    78,    31,    32,    -1,    65,    35,    26,    -1,
      38,    39,    40,     6,    -1,     8,    -1,     9,    10,    47,
      13,    -1,    -1,    -1,    -1,    -1,    -1,    45,    46,    57,
      48,    49,    60,    25,    -1,    -1,    29,    65,    56,    57,
      -1,    33,    35,    71,    72,    73,    39,    39,    76,    77,
      68,    -1,    -1,     6,    -1,     8,    -1,    50,    50,    -1,
      13,    53,    -1,     6,    57,     8,    -1,    60,    61,    12,
      -1,    -1,    65,    65,    -1,    -1,    29,    -1,    71,    72,
      73,    24,    35,    76,    77,    78,    39,    -1,    -1,    -1,
      -1,    -1,    35,    36,    -1,    -1,    39,    50,    41,    -1,
      43,    -1,    -1,    -1,    57,    -1,    -1,    60,    61,     3,
       4,    -1,    65,    -1,    57,    58,    59,    60,    71,    72,
      73,    -1,    65,    76,    77,    78,    -1,    -1,    71,    72,
      73,    -1,    -1,    76,    77,     6,    -1,     8,    -1,    -1,
      -1,    12,    -1,    -1,     6,    -1,     8,    -1,    -1,    -1,
      12,    45,    46,    -1,    48,    49,    -1,    -1,    -1,    -1,
      -1,    -1,    56,    57,    35,    36,    -1,    -1,    39,    -1,
      41,    -1,    43,    35,    36,    -1,    -1,    39,    -1,    41,
      -1,    43,    -1,    -1,    -1,    -1,    57,    58,    59,    60,
      -1,    -1,    -1,    -1,    65,    57,    58,    59,    60,    -1,
      71,    72,    73,    65,    -1,    76,    77,    -1,    -1,    71,
      72,    73,    -1,    -1,    76,    77,     6,    -1,     8,    -1,
      -1,    -1,    -1,    13,    -1,     6,    -1,     8,     9,    -1,
      -1,    -1,    -1,    -1,    -1,    25,    14,    15,    -1,    17,
      18,    19,    20,    21,    22,    35,    -1,    -1,    -1,    39,
      -1,    -1,    -1,    -1,    35,    -1,    -1,    -1,    39,    -1,
       6,    -1,     8,     9,     3,     4,    -1,    57,    -1,    50,
      60,    -1,    -1,    -1,    -1,    65,    57,    -1,    -1,    60,
      -1,    71,    72,    73,    65,    -1,    76,    77,    -1,    35,
      71,    72,    73,    39,    -1,    76,    77,    -1,    -1,     6,
      -1,     8,    -1,    -1,    -1,    -1,    45,    46,     6,    48,
       8,    57,    -1,    -1,    60,    -1,    -1,    56,    57,    65,
      -1,    -1,    -1,    -1,    -1,    71,    72,    73,    35,    -1,
      76,    77,    39,    -1,    -1,    -1,    -1,    35,    -1,    -1,
      -1,    39,    -1,    50,    -1,    -1,    -1,     6,    -1,     8,
      57,    -1,    50,    60,    13,    -1,    -1,    -1,    65,    57,
      -1,    -1,    60,    -1,    71,    72,    73,    65,    -1,    76,
      77,    -1,    -1,    71,    72,    73,    35,    -1,    76,    77,
      39,    -1,     6,    -1,     8,    -1,    -1,    -1,    -1,    -1,
      -1,     6,    -1,     8,     9,    -1,    -1,    -1,    57,    -1,
      -1,    60,    -1,    -1,    -1,    -1,    65,    -1,    -1,    -1,
      -1,    35,    71,    72,    73,    39,    -1,    76,    77,    -1,
      35,    -1,     3,     4,    39,    -1,    50,    -1,    -1,    -1,
       6,    -1,     8,    57,    -1,    -1,    60,    -1,    -1,    -1,
      -1,    65,    57,    -1,    -1,    60,    -1,    71,    72,    73,
      65,    -1,    76,    77,    -1,    -1,    71,    72,    73,    35,
      -1,    76,    77,    39,    45,    46,    -1,    48,    49,    -1,
       3,     4,     5,    -1,    -1,    56,    57,    -1,    -1,    12,
      -1,    57,    -1,    16,    60,    -1,    -1,    68,    -1,    65,
      -1,    -1,    -1,    26,    27,    71,    72,    73,    31,    32,
      76,    77,    -1,    36,    -1,    38,    -1,    40,    41,    -1,
      43,    -1,    45,    46,    47,    48,    49,    -1,     3,     4,
       5,    -1,    -1,    56,    57,    58,    59,    12,    -1,    -1,
      -1,    16,    -1,    -1,    -1,    68,    -1,    -1,    -1,    -1,
      -1,    26,    27,    -1,    -1,    -1,    31,    32,    -1,    -1,
      -1,    36,    -1,    38,    -1,    40,    41,    -1,    43,    -1,
      45,    46,    47,    48,    49,    -1,     3,     4,     5,    -1,
      -1,    56,    57,    58,    59,    12,    -1,    -1,    -1,    -1,
      -1,    -1,    -1,    68,    -1,    -1,    -1,    -1,    -1,    26,
      27,     3,     4,    -1,    31,    32,    -1,     9,    -1,    36,
      -1,    38,    -1,    40,    41,    -1,    43,    -1,    45,    46,
      47,    48,    49,    25,    26,     3,     4,    -1,    -1,    56,
      57,    58,    59,     9,    10,    -1,    -1,    -1,    -1,    -1,
      -1,    68,    -1,    45,    46,    -1,    48,    49,    26,    25,
      -1,    -1,    -1,    -1,    56,    57,    -1,    33,    -1,    -1,
      -1,    -1,    -1,    39,    -1,    -1,    68,    45,    46,    -1,
      48,    49,    -1,    -1,    50,    -1,    -1,    53,    56,    57,
      -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    65,
      68
  };

  const unsigned char
  parser::yystos_[] =
  {
       0,    69,    70,    80,    81,    73,    83,   131,     0,     1,
       6,     8,    11,    12,    13,    24,    28,    29,    33,    34,
      35,    36,    39,    41,    42,    43,    44,    54,    55,    57,
      58,    59,    60,    61,    62,    65,    67,    71,    72,    74,
      75,    76,    77,    78,    82,    83,    87,    93,    94,    95,
      96,    98,   103,   113,   114,   117,   121,   124,     5,    25,
      83,    57,    83,    87,    83,    87,    36,    57,    83,    93,
      25,   122,   123,    40,    76,    50,    78,    87,    94,   112,
      87,    89,    90,    91,    36,    36,    13,    25,    87,    13,
      57,    73,    83,    87,    73,    87,    88,    25,   123,    25,
      25,    24,    78,    93,    39,     3,     4,     5,    16,    26,
      27,    31,    32,    38,    40,    45,    46,    47,    48,    49,
      56,    57,    68,    92,   113,     9,    10,    53,    65,   119,
      14,    15,    17,    18,    19,    20,    21,    22,    97,    97,
      36,    87,    92,   109,    25,    33,    25,    33,     6,     8,
      35,    39,    57,    60,    65,    71,    76,    83,    84,    39,
      39,     5,    16,    90,   115,   116,    83,    39,     9,    25,
      36,    78,    87,    94,   103,   108,   110,   117,   118,    73,
      25,    78,    92,     9,   101,    50,    53,    10,    52,    53,
      50,    87,   126,   129,    50,   126,   130,    87,     9,    25,
      73,    73,    56,    39,    25,    39,    53,    65,    37,    37,
      36,    24,    93,    91,    87,    87,    13,    87,    87,    87,
      87,    87,    87,    87,    87,    87,   113,   109,    94,    99,
     120,   120,    87,    95,    95,    96,    96,    50,    90,   111,
      87,    25,   123,   123,    83,    84,    84,    85,    86,    84,
      84,    39,     3,     4,    45,    46,    48,    49,    56,    57,
      68,    91,    91,    84,    87,     9,    50,    53,    39,    91,
      25,   123,    50,    94,   106,   107,    78,    87,   110,    92,
     108,     9,    10,    25,    53,    36,   109,    10,    25,    53,
      25,    53,    25,    53,    32,    99,   100,    94,    87,    90,
      25,     6,    10,   125,    50,    53,    25,   125,    50,    53,
       9,    25,   123,    56,    56,    71,    91,    73,   132,   133,
      87,   126,   126,   116,    36,    52,    87,   109,    10,   119,
      94,    94,     9,    50,    53,    39,    10,    52,    65,    86,
      84,    84,    84,    84,    84,    84,    84,    84,    84,    52,
      52,    25,    96,    90,    91,    52,   101,    50,    53,   110,
      92,    10,    25,    53,   108,   109,   100,     9,    50,    89,
     104,   105,    25,   101,    87,    89,     9,   128,    25,   126,
     128,    25,   126,   123,    71,    71,    25,    52,    10,    52,
     125,   125,    50,   116,    94,   120,     9,    10,   102,   102,
      94,    90,    86,    84,    52,   101,     9,    52,   106,    10,
      25,    53,   109,   100,   101,    50,    53,    94,   127,   125,
     125,    25,    25,    73,    25,    51,    51,    50,    94,    99,
     119,   101,     9,    52,    96,   104,    10,   128,   128,   102,
      94,   101,    94,   101
  };

  const unsigned char
  parser::yyr1_[] =
  {
       0,    79,    80,    80,    81,    81,    82,    83,    84,    84,
      84,    84,    84,    84,    84,    84,    84,    84,    84,    84,
      84,    84,    84,    84,    84,    84,    84,    84,    85,    85,
      86,    86,    87,    87,    87,    87,    87,    87,    87,    87,
      87,    87,    87,    87,    87,    87,    87,    87,    87,    87,
      87,    87,    87,    87,    87,    88,    88,    89,    89,    90,
      90,    91,    91,    92,    92,    92,    92,    92,    92,    92,
      93,    93,    93,    93,    94,    94,    94,    94,    94,    94,
      94,    95,    95,    95,    95,    96,    96,    96,    97,    97,
      97,    97,    97,    97,    98,    98,    99,    99,   100,   100,
     101,   101,   102,   102,   103,   103,   103,   103,   103,   104,
     104,   105,   105,   106,   107,   107,   108,   108,   108,   108,
     109,   109,   109,   110,   110,   110,   111,   111,   112,   112,
     113,   113,   113,   113,   114,   114,   114,   115,   115,   116,
     116,   117,   117,   117,   118,   119,   119,   120,   120,   120,
     121,   121,   121,   121,   122,   122,   122,   122,   122,   122,
     122,   122,   122,   122,   122,   123,   123,   123,   123,   123,
     123,   124,   124,   124,    82,    82,    82,    82,    82,    82,
      82,   125,   125,   126,   126,   127,   127,   128,   128,   128,
      82,    82,   129,   129,   130,   130,    82,    82,    82,    82,
      82,    82,    82,    82,    82,    82,    82,    82,   131,    82,
      82,    82,    82,    82,   132,   132,   133,   133,    82,    82,
      82,    82,    82
  };

  const unsigned char
  parser::yyr2_[] =
  {
       0,     2,     2,     2,     2,     0,     2,     1,     3,     3,
       3,     3,     3,     3,     3,     3,     3,     2,     2,     3,
       4,     5,     3,     1,     1,     1,     1,     1,     1,     3,
       1,     0,     3,     3,     3,     3,     3,     3,     3,     3,
       3,     3,     2,     2,     3,     4,     5,     3,     1,     1,
       1,     1,     1,     1,     1,     1,     3,     1,     3,     1,
       0,     1,     3,     1,     1,     1,     1,     1,     1,     1,
       1,     4,     2,     5,     1,     1,     1,     2,     3,     3,
       1,     4,     4,     2,     1,     3,     3,     1,     1,     1,
       1,     1,     1,     1,     3,     3,     1,     3,     1,     0,
       2,     0,     2,     0,     1,     1,     1,     1,     1,     2,
       2,     1,     3,     2,     1,     3,     2,     3,     3,     4,
       1,     2,     0,     3,     4,     2,     6,     4,     2,     4,
       3,     4,     2,     3,     3,     4,     2,     4,     6,     1,
       0,     4,     5,     6,     3,     1,     1,     3,     4,     0,
       5,     5,     7,     3,     3,     3,     3,     3,     4,     4,
       5,     5,     3,     3,     0,     3,     3,     4,     5,     3,
       3,     1,     1,     1,     2,     3,     2,     2,     3,     3,
       2,     2,     0,     3,     1,     1,     3,     2,     1,     0,
       6,     6,     3,     5,     3,     5,     4,     4,     5,     5,
       5,     6,     2,     4,     3,     6,     5,     4,     3,     5,
       2,     2,     3,     5,     3,     1,     0,     1,     6,     3,
       4,     4,     3
  };



  // YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
  // First, the terminals, then, starting at \a yyntokens_, nonterminals.
  const char*
  const parser::yytname_[] =
  {
  "\"<EOF>\"", "error", "$undefined", "\"+\"", "\"&\"", "\"=\"", "\"@\"",
  "\"#base\"", "\"~\"", "\":\"", "\",\"", "\"#const\"", "\"#count\"",
  "\"$\"", "\"$+\"", "\"$-\"", "\"$*\"", "\"$<=\"", "\"$<\"", "\"$>\"",
  "\"$>=\"", "\"$=\"", "\"$!=\"", "\"#cumulative\"", "\"#disjoint\"",
  "\".\"", "\"..\"", "\"==\"", "\"#external\"", "\"#false\"",
  "\"#forget\"", "\">=\"", "\">\"", "\":-\"", "\"#include\"", "\"#inf\"",
  "\"{\"", "\"[\"", "\"<=\"", "\"(\"", "\"<\"", "\"#max\"",
  "\"#maximize\"", "\"#min\"", "\"#minimize\"", "\"\\\\\"", "\"*\"",
  "\"!=\"", "\"**\"", "\"?\"", "\"}\"", "\"]\"", "\")\"", "\";\"",
  "\"#show\"", "\"#showsig\"", "\"/\"", "\"-\"", "\"#sum\"", "\"#sum+\"",
  "\"#sup\"", "\"#true\"", "\"#program\"", "UBNOT", "UMINUS", "\"|\"",
  "\"#volatile\"", "\":~\"", "\"^\"", "\"<program>\"", "\"<define>\"",
  "\"<NUMBER>\"", "\"<ANONYMOUS>\"", "\"<IDENTIFIER>\"", "\"<PYTHON>\"",
  "\"<LUA>\"", "\"<STRING>\"", "\"<VARIABLE>\"", "\"not\"", "$accept",
  "start", "program", "statement", "identifier", "constterm",
  "consttermvec", "constargvec", "term", "unaryargvec", "ntermvec",
  "termvec", "argvec", "cmp", "atom", "literal", "csp_mul_term",
  "csp_add_term", "csp_rel", "csp_literal", "nlitvec", "litvec",
  "optcondition", "noptcondition", "aggregatefunction", "bodyaggrelem",
  "bodyaggrelemvec", "altbodyaggrelem", "altbodyaggrelemvec",
  "bodyaggregate", "upper", "lubodyaggregate", "headaggrelemvec",
  "altheadaggrelemvec", "headaggregate", "luheadaggregate", "ncspelemvec",
  "cspelemvec", "disjoint", "conjunction", "dsym", "disjunctionsep",
  "disjunction", "bodycomma", "bodydot", "head", "optimizetuple",
  "optimizeweight", "optimizelitvec", "optimizecond", "maxelemlist",
  "minelemlist", "define", "nidlist", "idlist", YY_NULLPTR
  };

#if YYDEBUG
  const unsigned short int
  parser::yyrline_[] =
  {
       0,   287,   287,   288,   292,   293,   299,   303,   311,   312,
     313,   314,   315,   316,   317,   318,   319,   320,   321,   322,
     323,   324,   325,   326,   327,   328,   329,   330,   336,   337,
     341,   342,   350,   351,   352,   353,   354,   355,   356,   357,
     358,   359,   360,   361,   362,   363,   364,   365,   366,   367,
     368,   369,   370,   371,   372,   378,   379,   386,   387,   391,
     392,   396,   397,   406,   407,   408,   409,   410,   411,   412,
     416,   417,   418,   419,   423,   424,   425,   426,   427,   428,
     429,   433,   434,   435,   436,   440,   441,   442,   446,   447,
     448,   449,   450,   451,   455,   456,   464,   465,   469,   470,
     474,   475,   479,   480,   484,   485,   486,   487,   488,   496,
     497,   501,   502,   508,   512,   513,   519,   520,   521,   522,
     526,   527,   528,   532,   533,   534,   542,   543,   547,   548,
     554,   555,   556,   557,   561,   562,   563,   570,   571,   575,
     576,   580,   581,   582,   589,   596,   597,   601,   602,   603,
     608,   609,   610,   611,   620,   621,   622,   623,   624,   625,
     626,   627,   628,   629,   630,   634,   635,   636,   637,   638,
     639,   643,   644,   645,   649,   650,   651,   652,   659,   660,
     661,   668,   669,   673,   674,   678,   679,   683,   684,   685,
     689,   690,   694,   695,   699,   700,   704,   705,   706,   707,
     714,   715,   716,   717,   718,   719,   720,   721,   728,   732,
     739,   740,   747,   748,   755,   756,   760,   761,   765,   766,
     773,   774,   775
  };

  // Print the state stack on the debug stream.
  void
  parser::yystack_print_ ()
  {
    *yycdebug_ << "Stack now";
    for (stack_type::const_iterator
           i = yystack_.begin (),
           i_end = yystack_.end ();
         i != i_end; ++i)
      *yycdebug_ << ' ' << i->state;
    *yycdebug_ << std::endl;
  }

  // Report on the debug stream that the rule \a yyrule is going to be reduced.
  void
  parser::yy_reduce_print_ (int yyrule)
  {
    unsigned int yylno = yyrline_[yyrule];
    int yynrhs = yyr2_[yyrule];
    // Print the symbols being reduced, and their result.
    *yycdebug_ << "Reducing stack by rule " << yyrule - 1
               << " (line " << yylno << "):" << std::endl;
    // The symbols being reduced.
    for (int yyi = 0; yyi < yynrhs; yyi++)
      YY_SYMBOL_PRINT ("   $" << yyi + 1 << " =",
                       yystack_[(yynrhs) - (yyi + 1)]);
  }
#endif // YYDEBUG

  // Symbol number corresponding to token number t.
  inline
  parser::token_number_type
  parser::yytranslate_ (int t)
  {
    static
    const token_number_type
    translate_table[] =
    {
     0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     1,     2,     3,     4,
       5,     6,     7,     8,     9,    10,    11,    12,    13,    14,
      15,    16,    17,    18,    19,    20,    21,    22,    23,    24,
      25,    26,    27,    28,    29,    30,    31,    32,    33,    34,
      35,    36,    37,    38,    39,    40,    41,    42,    43,    44,
      45,    46,    47,    48,    49,    50,    51,    52,    53,    54,
      55,    56,    57,    58,    59,    60,    61,    62,    63,    64,
      65,    66,    67,    68,    69,    70,    71,    72,    73,    74,
      75,    76,    77,    78
    };
    const unsigned int user_token_number_max_ = 333;
    const token_number_type undef_token_ = 2;

    if (static_cast<int>(t) <= yyeof_)
      return yyeof_;
    else if (static_cast<unsigned int> (t) <= user_token_number_max_)
      return translate_table[t];
    else
      return undef_token_;
  }

#line 23 "libgringo/src/input/nongroundgrammar.yy" // lalr1.cc:1155
} } } // Gringo::Input::NonGroundGrammar
#line 2823 "/home/sni/pjx/catkin_ws/src/ice/cling_wrap/libgringo/src/input/nongroundgrammar/grammar.cc" // lalr1.cc:1155
