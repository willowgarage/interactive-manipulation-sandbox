from ply import lex, yacc

tokens = (
    'SYMBOL',
    'LPAREN',
    'RPAREN',
    'WHITESPACE',
    )

t_SYMBOL = r'\w+'
t_LPAREN = r'\('
t_RPAREN = r'\)'
t_WHITESPACE = r'\s+'

def t_error(t):
    raise TypeError("Unknown text '%s'" % (t.value,))

lex.lex()

class QueryFunctionCall:
    def __init__(self, name, args):
        self.name = name
        self.args = args

    def __repr__(self):
        if len(self.args) == 0:
            return '(%s)' % self.name
        else:
            return '(%s %s)' % (self.name, ' '.join([str(arg) for arg in self.args]))

def p_query_function_call(p):
    '''
    query_function_call : LPAREN SYMBOL RPAREN
    query_function_call : LPAREN SYMBOL WHITESPACE query_function_call_arg_list RPAREN
    '''
    if len(p) == 4:
        p[0] = QueryFunctionCall(p[2], [])
    else:
        p[0] = QueryFunctionCall(p[2], p[4])

def p_query_function_call_arg_list(p):
    '''
    query_function_call_arg_list : query_function_call_arg
    query_function_call_arg_list : query_function_call_arg WHITESPACE query_function_call_arg_list
    '''
    if len(p) == 1:
        p[0] = []
    elif len(p) == 2:
        p[0] = [p[1]]
    else:
        p[0] = [p[1]] + p[3]

def p_query_function_call_arg(p):
    '''
    query_function_call_arg : SYMBOL
    query_function_call_arg : query_function_call
    '''
    p[0] = p[1]

def p_error(p):
    if p is None:
        print 'Unable to parse query'
        return
    print "Syntax error at '%s'" % p.value

yacc.yacc()

def parse_query(query_str):
    return yacc.parse(query_str)


if __name__ == '__main__':
    test_strings = [
        '(foo)',
        '(foo a)',
        '(foo a b)',
        '(foo a b (bar c))'
        ]

    for test_str in test_strings:
        parse_tree = parse_query(test_str)
        print '%s: %s' % (test_str, parse_tree)
