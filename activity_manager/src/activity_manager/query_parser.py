from ply import lex, yacc

tokens = (
    'SYMBOL',
    'LPAREN',
    'RPAREN',
    'COMMA',
    'AND',
    'OR'
    )

t_SYMBOL = r'\w+'
t_LPAREN = r'\('
t_RPAREN = r'\)'
t_COMMA = r','
t_AND = r'\&'
t_OR = r'\|'

def t_error(t):
    raise TypeError("Unknown text '%s'" % (t.value,))

lex.lex()

class QueryFunctionCall:
    def __init__(self, name, args):
        self.name = name
        self.args = args

    def __repr__(self):
        return '%s(%s)' % (self.name, str(self.args))

class QueryVariable:
    def __init__(self, name):
        self.name = name
        self.val = None

    def __repr__(self):
        return str(self.name)

class Conjunction:
    def __init__(self, lval, rval):
        self.lval = lval
        self.rval = rval

    def __repr__(self):
        return '(%s & %s)' % (self.lval, self.rval)

class Disjunction:
    def __init__(self, lval, rval):
        self.lval = lval
        self.rval = rval

    def __repr__(self):
        return '(%s | %s)' % (self.lval, self.rval)

def p_expression(p):
    '''
    expression : function_call
           | paren_group
           | conjunction
           | disjunction
    '''
    if len(p) == 2:
        p[0] = p[1]
    else:
        p[0] = p[2]

def p_paren_group(p):
    '''
    paren_group : LPAREN expression RPAREN
    '''
    p[0] = p[2]

def p_conjunction(p):
    '''
    conjunction : expression AND function_call
    '''
    p[0] = Conjunction(p[1], p[3])

def p_disjunction(p):
    '''
    disjunction : expression OR function_call
    '''
    p[0] = Disjunction(p[1], p[3])    

def p_function_call(p):
    "function_call : SYMBOL LPAREN query_function_call_args RPAREN"
    p[0] = QueryFunctionCall(p[1], p[3])

def p_query_function_call_args(p):
    '''
    query_function_call_args :
    query_function_call_args : SYMBOL
    query_function_call_args : SYMBOL COMMA query_function_call_args
    '''
    if len(p) == 1:
        p[0] = []
    elif len(p) == 2:
        p[0] = [QueryVariable(p[1])]
    else:
        p[0] = [QueryVariable(p[1])] + p[3]

def p_error(p):
    if p is None:
        print 'Unable to parse query'
        return
    print "Syntax error at '%s'" % p.value

yacc.yacc()

def parse_query(query_str):
    # remove spaces
    cleaned_query_str = query_str.translate(None, ' ')
    return yacc.parse(cleaned_query_str)


if __name__ == '__main__':
    test_strings = [
        '(foo())',
        '(foo(a))',
        '(foo(a,b))',
        '( (foo(a,b) & bar(c)) | foo(d) )']

    for test_str in test_strings:
        parse_tree = parse_query(test_str)
        print '%s: %s' % (test_str, parse_tree)
